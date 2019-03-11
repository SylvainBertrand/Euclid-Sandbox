package us.ihmc.euclid.visualizers;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class ConvexPolytope3DVisualizer extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(600, 400);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.005, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.1);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, -1.0, Color.WHEAT);

      ConvexPolytope3DTroublesomeDataset dataset = new ConvexPolytope3DTroublesomeDataset_20190310_190049();

      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(dataset.getPointsBeforeIssue()),
                                                               dataset.getConstructionEpsilon());
      //      convexPolytope3D.addVertex(dataset.getTroublesomePoint());
      view3dFactory.addNodeToView(generatePointMesh(dataset.getTroublesomePoint(), Color.BLACK, 0.1));
      Vector3D supportDirection = new Vector3D();
      supportDirection.sub(dataset.getTroublesomePoint(), convexPolytope3D.getCentroid());
      view3dFactory.addNodeToView(generatePointMesh(convexPolytope3D.getSupportingVertex(supportDirection), Color.YELLOWGREEN, 0.1));
      view3dFactory.addNodeToView(generatePointMesh(convexPolytope3D.getCentroid(), Color.DARKVIOLET, 0.1));
      view3dFactory.addNodeToView(generateFace3DsMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(generateFace3DsNormalMesh(convexPolytope3D.getFaces()));

      Face3DReadOnly naiveClosestFace = convexPolytope3D.getFaces().stream().sorted((f1, f2) -> Double.compare(f1.distance(dataset.getTroublesomePoint()),
                                                                                                               f2.distance(dataset.getTroublesomePoint())))
                                                        .findFirst().get();
      view3dFactory.addNodeToView(generateFace3DsMesh(Collections.singletonList(naiveClosestFace), Color.DARKRED));
      //      view3dFactory.addNodeToView(generateFace3DsMesh(Collections.singletonList(convexPolytope3D.getClosestFace(dataset.getTroublesomePoint())), Color.GREEN));

      //      System.out.println(naiveClosestFace.signedDistanceToPlane(dataset.getTroublesomePoint()));
      //      System.out.println(convexPolytope3D.getClosestFace(dataset.getTroublesomePoint()).signedDistanceToPlane(dataset.getTroublesomePoint()));

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

   public static <F extends Face3DReadOnly> Node generateFace3DsMesh(List<F> faces)
   {
      Group group = new Group();

      for (F face : faces)
      {
         List<Point3D> cwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         cwFaceVertices.add(cwFaceVertices.get(0));
         cwFaceVertices.add(0, new Point3D(face.getCentroid()));
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         ccwFaceVertices.add(ccwFaceVertices.get(0));
         ccwFaceVertices.add(0, new Point3D(face.getCentroid()));

         MeshView outsideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(ccwFaceVertices)));
         outsideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println(face.toString()));
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         outsideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 0.7)));
         group.getChildren().add(outsideFaceNode);

         //         MeshView insideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(cwFaceVertices)));
         //         insideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println(face.toString()));
         //         insideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0).darker()));
         //         group.getChildren().add(insideFaceNode);
      }

      return group;
   }

   public static Node generateFace3DsMesh(List<? extends Face3DReadOnly> faces, Color color)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Face3DReadOnly face : faces)
      {
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         meshBuilder.addMesh(MeshDataGenerator.Polygon(ccwFaceVertices));
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node generateFace3DsNormalMesh(List<? extends Face3DReadOnly> faces)
   {
      Group group = new Group();

      for (Face3DReadOnly face : faces)
      {
         double scale = Math.max(0.003, face.getEdges().stream().mapToDouble(HalfEdge3DReadOnly::length).max().getAsDouble());
         double height = 0.010 * scale;
         double radius = 0.005 * scale;
         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(face.getNormal());
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         MeshDataHolder cone = MeshDataGenerator.Cone(height, radius, 32);
         cone = MeshDataHolder.rotate(cone, orientation);
         cone = MeshDataHolder.translate(cone, face.getCentroid());
         MeshView normalNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(cone));
         normalNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println(face.toString()));
         normalNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0)));
         group.getChildren().add(normalNode);
      }

      return group;
   }

   public static Node generatePointMesh(Tuple3DReadOnly position, Color color, double size)
   {
      Sphere sphere = new Sphere();
      sphere.setRadius(size);
      sphere.setMaterial(new PhongMaterial(color));
      sphere.setTranslateX(position.getX());
      sphere.setTranslateY(position.getY());
      sphere.setTranslateZ(position.getZ());
      return sphere;
   }

   public static Node generateHalfEdge3DsMesh(Collection<? extends HalfEdge3DReadOnly> edges, Color color)
   {
      Group group = new Group();

      for (HalfEdge3DReadOnly edge : edges)
      {
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Line(edge.getOrigin(), edge.getDestination(),
                                                                                                             0.00001)));
         edgeNode.setMaterial(new PhongMaterial(color));
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println(edge.toString()));
         group.getChildren().add(edgeNode);
      }

      return group;
   }

   public static Color nextColor(Random random)
   {
      return Color.hsb(EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0), 0.9, 0.9);
   }

   public class ConvexPolytope3DTroublesomeDataset_20190310_190049 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190310_190049()
      {
         pointsBeforeIssue.add(new Point3D(5.50959228909476600000, -7.37482512292480000000, -4.10818520693522900000));
         pointsBeforeIssue.add(new Point3D(3.26408212934210870000, -7.39707117672959000000, -1.22355245643036800000));
         pointsBeforeIssue.add(new Point3D(7.53014890474895500000, -6.27335116777345600000, 0.17761833927299975000));
         pointsBeforeIssue.add(new Point3D(7.00957333504974900000, -4.66027284035378300000, -8.73680095021806300000));
         pointsBeforeIssue.add(new Point3D(8.14949598684792200000, -6.49543245326867800000, -5.37935993441628200000));
         pointsBeforeIssue.add(new Point3D(8.35662907631226300000, 1.84543837022117600000, -5.39953823637316200000));
         pointsBeforeIssue.add(new Point3D(2.96308939191737770000, 1.76142157519687900000, -8.77596291853211800000));
         pointsBeforeIssue.add(new Point3D(-0.93580603815346650000, -4.92396725159303900000, -0.11532085278145665000));
         pointsBeforeIssue.add(new Point3D(-0.76463657796814740000, 1.03347633175171080000, 0.80886057878909770000));
         pointsBeforeIssue.add(new Point3D(-0.28129539962552474000, -3.53342168472010170000, 0.49480723626103007000));
         pointsBeforeIssue.add(new Point3D(8.25634207586230100000, -6.37824459900945900000, -1.87629904675217580000));
         pointsBeforeIssue.add(new Point3D(-1.42124933586592480000, -4.42321621569930000000, -8.93592189307561400000));
         pointsBeforeIssue.add(new Point3D(-0.98563205874393670000, 1.80198735635120320000, -9.09616868123428900000));
         pointsBeforeIssue.add(new Point3D(-1.57445104301922000000, -6.43311386664316500000, -5.34389229822706300000));
         pointsBeforeIssue.add(new Point3D(-0.68926046477887400000, -7.56637055170942000000, -6.83078977271695800000));
         pointsBeforeIssue.add(new Point3D(1.69274158458135160000, -7.09522813808260600000, -8.82078837159815900000));
         pointsBeforeIssue.add(new Point3D(5.26469169713043900000, -7.39208674608942600000, -4.21398782657623300000));
         pointsBeforeIssue.add(new Point3D(-1.36613299846150400000, -6.70484031508754900000, -2.02753288121639040000));
         pointsBeforeIssue.add(new Point3D(6.76141818392682700000, -5.48910896683029000000, -9.05252572352598300000));
         pointsBeforeIssue.add(new Point3D(-1.48478727745275300000, -5.48177742169217550000, -1.08538604047731720000));
         pointsBeforeIssue.add(new Point3D(1.96062734221581360000, -1.94952655262267700000, -9.07837819535282400000));
         pointsBeforeIssue.add(new Point3D(4.13367820793450100000, 2.38250239054842600000, 0.26226012065547977000));
         pointsBeforeIssue.add(new Point3D(1.46656243159462420000, 2.35701339241588850000, -6.82990764153806750000));
         pointsBeforeIssue.add(new Point3D(0.73320119057805310000, -7.00643772944042400000, -0.94710004057561110000));
         pointsBeforeIssue.add(new Point3D(6.51576151610400000000, 1.45489109336699580000, -0.75991614441446490000));
         pointsBeforeIssue.add(new Point3D(6.21350164476770850000, -0.81771773142268160000, 0.16804616890483004000));
         pointsBeforeIssue.add(new Point3D(3.46824065760577300000, -3.92823742831175070000, 0.79287770829049190000));
         pointsBeforeIssue.add(new Point3D(2.17126713572093170000, 2.34231187412905600000, -2.38050654586149500000));
         troublesomePoint.set(6.28071978048610700000, -13.20013773779965400000, -5.21906497359370400000);
         constructionEpsilon = 1.0E-10;
      }
   }

   public static void main(String[] args)
   {
      launch();
   }
}
