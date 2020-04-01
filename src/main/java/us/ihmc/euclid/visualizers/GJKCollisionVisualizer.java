package us.ihmc.euclid.visualizers;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.EuclidShapeCollisionTools;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class GJKCollisionVisualizer extends Application
{
   private final GilbertJohnsonKeerthiCollisionDetector gjkDetector = new GilbertJohnsonKeerthiCollisionDetector();

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(600, 400);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.001, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.1);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, -1.0, Color.WHEAT);

      PointShape3D shapeA = new PointShape3D(-0.78997519398227170000, 0.13845592612248003000, -0.17886948344144193000);
      RotationMatrix orientation = new RotationMatrix();
      orientation.setUnsafe(0.49334021254769855000,
                            -0.42974327389421430000,
                            0.75626460529769210000,
                            0.01549819802185678500,
                            0.87363781569598780000,
                            0.48632990124402060000,
                            -0.86969836181371940000,
                            -0.22820535823820137000,
                            0.43766091204253926000);
      Ramp3D shapeB = new Ramp3D(new Point3D(-0.87039038842219820000, -0.26804467372865260000, 0.12297677487724501000),
                                 orientation,
                                 0.37564853798406050000,
                                 0.15587311579881924000,
                                 0.72434329526309900000);

      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      //      gjkDetector.evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB, result);
      EuclidShapeCollisionTools.evaluatePointShape3DRamp3DCollision(shapeA, shapeB, result);
      System.out.println("Expected: " + result);
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(result.getPointOnA(), Color.ORANGE, 0.01));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(result.getPointOnB(), Color.ORANGERED, 0.01));

      EuclidShape3DCollisionResult gjkResult = gjkDetector.evaluateCollision(shapeB, shapeA);
      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(gjkDetector.getSimplex().getVertices()), 1.0e-12);
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generateFace3DsMesh(convexPolytope3D.getFaces()));
      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generateFace3DsNormalMesh(convexPolytope3D.getFaces()));
      System.out.println("Actual: " + gjkResult);

      view3dFactory.addNodeToView(generateShape3DMesh(shapeA, Color.AQUAMARINE.deriveColor(0, 1, 1, 0.7)));
      view3dFactory.addNodeToView(generateShape3DMesh(shapeB, Color.CORNFLOWERBLUE.deriveColor(0, 1, 1, 0.7)));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(gjkResult.getPointOnA(), Color.AQUAMARINE, 0.01));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(gjkResult.getPointOnB(), Color.CADETBLUE, 0.01));

      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(new Point3D(0.7241694221106871000, 0.4841639275556230400, 0.7683243967949757000),
      //                                                                               Color.BLACK,
      //                                                                               0.01));

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
   }

   @Override
   public void stop()
   {
      Platform.exit();
   }

   public static Node generateTetrahedronsMesh(List<? extends Point3DReadOnly> points, Color color, double size)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Point3DReadOnly point : points)
      {
         meshBuilder.addTetrahedron(size, point);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node generateShape3DMesh(Shape3DReadOnly shape3D, Color color)
   {
      if (shape3D instanceof PointShape3DReadOnly)
         return generatePointShape3DMesh((PointShape3DReadOnly) shape3D, color);
      if (shape3D instanceof Sphere3DReadOnly)
         return generateSphere3DMesh((Sphere3DReadOnly) shape3D, color);
      if (shape3D instanceof Box3DReadOnly)
         return generateBox3DMesh((Box3DReadOnly) shape3D, color);
      if (shape3D instanceof Ramp3DReadOnly)
         return generateRamp3DMesh((Ramp3DReadOnly) shape3D, color);
      if (shape3D instanceof Ellipsoid3DReadOnly)
         return generateEllipsoid3DMesh((Ellipsoid3DReadOnly) shape3D, color);
      throw new UnsupportedOperationException("Unsupported shape " + shape3D);
   }

   public static Node generatePointShape3DMesh(PointShape3DReadOnly pointShape3D, Color color)
   {
      return ConvexPolytope3DVisualizer.generatePointMesh(pointShape3D, color, 0.01);
   }

   public static Node generateSphere3DMesh(Sphere3DReadOnly sphere3D, Color color)
   {
      return ConvexPolytope3DVisualizer.generatePointMesh(sphere3D.getPosition(), color, sphere3D.getRadius());
   }

   public static Node generateBox3DMesh(Box3DReadOnly box3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Cube(box3D.getSizeX(), box3D.getSizeY(), box3D.getSizeZ(), true);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(box3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, box3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node generateRamp3DMesh(Ramp3DReadOnly ramp3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Wedge(ramp3D.getSizeX(), ramp3D.getSizeY(), ramp3D.getSizeZ());
      mesh = MeshDataHolder.translate(mesh, (float) (0.5 * ramp3D.getSizeX()), 0.0f, 0.0f);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(ramp3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, ramp3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node generateEllipsoid3DMesh(Ellipsoid3DReadOnly ellipsoid3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Ellipsoid(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ(), 128, 128);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(ellipsoid3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, ellipsoid3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node generateCylinder3DMesh(Cylinder3DReadOnly cylinder3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Cylinder(cylinder3D.getRadius(), cylinder3D.getLength(), 128);
      mesh = MeshDataHolder.rotate(mesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinder3D.getAxis()));
      mesh = MeshDataHolder.translate(mesh, cylinder3D.getBottomCenter());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static ConvexPolytope3D fromSupportingVertices(Random random, int numberOfSamples, SupportingVertexHolder supportingVertexHolder,
                                                         double constructionEpsilon)
   {
      List<Point3DReadOnly> supportingVertices = IntStream.range(0, numberOfSamples)
                                                          .mapToObj(i -> supportingVertexHolder.getSupportingVertex(EuclidCoreRandomTools.nextVector3D(random)))
                                                          .collect(Collectors.toList());

      try
      {
         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(supportingVertices), constructionEpsilon);
         return convexPolytope3D;
      }
      catch (Exception e1)
      {
         for (int i = 0; i < supportingVertices.size(); i++)
         {
            try
            {
               new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(supportingVertices.subList(0, i + 1)), constructionEpsilon);
            }
            catch (Exception e2)
            {
               System.out.println(ConvexPolytope3DTroublesomeDataset.generateDatasetAsString(supportingVertices.subList(0, i),
                                                                                             supportingVertices.get(i),
                                                                                             constructionEpsilon));
               ThreadTools.sleep(500);
               break;
            }
         }
         throw e1;
      }
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
