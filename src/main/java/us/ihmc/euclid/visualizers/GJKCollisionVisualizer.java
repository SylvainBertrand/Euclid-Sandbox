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
import us.ihmc.euclid.shape.collision.EuclidShapeCollisionTools;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class GJKCollisionVisualizer extends Application
{

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

      Sphere3D shapeA = new Sphere3D(new Point3D(-0.1165512189637705200, -0.3018840659228632000, -0.1381164041574600700), 0.6826358574282498000);
      Cylinder3D shapeB = new Cylinder3D(new Point3D(-0.7285267466333849000, 0.6687589459382919000, 0.6766316564913009000),
                                         new Vector3D(-0.7874341659384139000, -0.6048414317347025000, -0.1188035217070166200), 0.1818974369960153500,
                                         0.7214674168888396000);

      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      EuclidShapeCollisionTools.evaluateSphere3DCylinder3DCollision(shapeA, shapeB, result);
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(result.getPointOnB(), Color.GREEN, 0.01));

      GilbertJohnsonKeerthiCollisionDetector gjkDetector = new GilbertJohnsonKeerthiCollisionDetector();
      EuclidShape3DCollisionResult gjkResult = gjkDetector.evaluateCollision(shapeA, shapeB);
      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(gjkDetector.getSimplex().getVertices()), 1.0e-12);
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generateFace3DsMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generateFace3DsNormalMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(gjkResult.getPointOnB(), Color.BLUE, 0.01));
      System.out.println("Distance: " + gjkResult.getSignedDistance() + ", " + result.getSignedDistance() + ", "
            + gjkResult.getPointOnA().distance(gjkResult.getPointOnB()));
      System.out.println("PointOnA: " + gjkResult.getPointOnA() + ", " + result.getPointOnA());
      System.out.println("PointOnB: " + gjkResult.getPointOnB() + ", " + result.getPointOnB());

      //      view3dFactory.addNodeToView(generateSphere3DMesh(shapeA, Color.RED.deriveColor(0, 1, 1, 0.7)));
      //      view3dFactory.addNodeToView(generateCylinder3DMesh(shapeB, Color.PURPLE.deriveColor(0, 1, 1, 0.7)));
      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(gjkResult.getPointOnB(), Color.BLUE, 0.01));
      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(gjkResult.getPointOnA(), Color.ALICEBLUE, 0.01));

      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(new Point3D(0.3103257110607079400, -0.2836069351142044300,
                                                                                           0.2592171748486601400),
                                                                               Color.BLACK, 0.01));

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
               System.out.println(ConvexPolytope3DTroublesomeDataset.generateDatasetAsString(supportingVertices.subList(0, i), supportingVertices.get(i),
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
