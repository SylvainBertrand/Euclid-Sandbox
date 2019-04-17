package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
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
import javafx.stage.Stage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3DTroublesomeDatasetLibrary.ConvexPolytope3DTroublesomeDataset_20190323_213507;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3DTroublesomeDatasetLibrary.DatasetGJKNullPointerExceptionBug1Simplified;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3DTroublesomeDatasetLibrary.DatasetGJKNullPointerExceptionBug4Simplified;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class ConvexPolytope3DVisualizer extends Application
{
   private final ConvexPolytope3DTroublesomeDataset dataset = new EPA_Dataset();
   private final ConvexPolytope3D convexPolytope3D = dataset.getConvexPolytope3D();
   private final Point3D troublesomePoint = dataset.getTroublesomePoint();
   private final double constructionEpsilon = dataset.getConvexPolytope3D().getConstructionEpsilon();

   public class EPA_Dataset extends ConvexPolytope3DTroublesomeDataset
   {
      public EPA_Dataset()
      {
         double constructionEpsilon = 0.0;
         Vertex3D v0 = new Vertex3D( 0.06160041628342216600,  0.45457028156296264000,  0.20609897863669868000 );
         Vertex3D v1 = new Vertex3D( 0.18045052005031104000,  0.36623995385144714000, -0.04369899248186826000 );
         Vertex3D v2 = new Vertex3D( 0.66745033364347500000, -0.42798370867877900000,  0.94883253208013140000 );
         Vertex3D v3 = new Vertex3D(-1.21298422578573950000,  0.19131372581296580000,  0.14495345121573910000 );
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)), new Vector3D( 0.28606609434297940000,  0.23961404005679470000,  0.05137671157570477400 ), constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v3), new HalfEdge3D(v3, v1), new HalfEdge3D(v1, v0)), new Vector3D(-0.06035994903482430000,  0.32565580988662370000, -0.14387254809823460000 ), constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v0)), new Vector3D(-0.24949370633027800000,  0.90963176763352920000,  1.28438372430460860000 ), constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v2), new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v3)), new Vector3D( 0.02378756102212287000, -1.47490161757694740000, -1.19188788778207840000 ), constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
         troublesomePoint.set(-0.39587000462122590000, -0.09761499076555857000,  0.94708292196562380000 );
      }
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(600, 400);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.0001, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.5);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, -1.0, Color.WHEAT);

      //      Point3D focusPoint = troublesomePoint;
      //      cameraController.getTranslate().setX(focusPoint.getX());
      //      cameraController.getTranslate().setY(focusPoint.getY());
      //      cameraController.getTranslate().setZ(focusPoint.getZ());
      //      convexPolytope3D.addVertex(troublesomePoint);

      view3dFactory.addNodeToView(generateFace3DsMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(generateFace3DsNormalMesh(convexPolytope3D.getFaces()));

      view3dFactory.addNodeToView(generateMultilineMesh(Arrays.asList(new Point3D(0.4789278257837637000, -1.0709997264316717000, -0.3921167138841577000),
                                                                      new Point3D(-0.5706011857203901000, -1.5817801098497340000, -0.3254900162247995600),
                                                                      new Point3D(-0.1226494165834033400, 0.4219890315821706000, 0.1456226472088930600),
                                                                      new Point3D(0.1030819243869309800, -0.3550176957114557000, -0.1224997543555419500)),
                                                        true, Color.ORANGE, 0.005));

      view3dFactory.addNodeToView(generatePointMesh(troublesomePoint, Color.BLACK, 0.1));

      convexPolytope3D.getFaces()
                      .forEach(f -> System.out.println(convexPolytope3D.getFaces().indexOf(f) + "\t" + f.distance(new Point3D()) * f.distance(new Point3D())));

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
         outsideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         outsideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 0.5)));
         group.getChildren().add(outsideFaceNode);

         MeshView insideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(cwFaceVertices)));
         insideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         insideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0).darker()));
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

      for (int i = 0; i < faces.size(); i++)
      {
         Face3DReadOnly face = faces.get(i);
         double scale = Math.max(0.00003, face.getEdges().stream().mapToDouble(HalfEdge3DReadOnly::length).max().getAsDouble());
         double height = 0.010 * scale;
         double radius = 0.005 * scale;
         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(face.getNormal());
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         MeshDataHolder cone = MeshDataGenerator.Cone(height, radius, 32);
         cone = MeshDataHolder.rotate(cone, orientation);
         cone = MeshDataHolder.translate(cone, face.getCentroid());
         MeshView normalNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(cone));
         normalNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         normalNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0)));
         group.getChildren().add(normalNode);
      }

      return group;
   }

   public static Node generatePointMesh(Tuple3DReadOnly position, Color color, double size)
   {
      MeshDataHolder sphereMeshData = MeshDataHolder.translate(MeshDataGenerator.Sphere(size, 64, 64), position);
      MeshView node = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(sphereMeshData));
      node.setMaterial(new PhongMaterial(color));
      return node;
   }

   public static Node generateHalfEdge3DsMesh(List<? extends HalfEdge3DReadOnly> edges, Color color, double width)
   {
      Group group = new Group();

      if (edges == null)
         return group;

      for (HalfEdge3DReadOnly edge : edges)
      {
         MeshDataHolder line = MeshDataGenerator.Line(edge.getOrigin(), edge.getDestination(), width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + edges.indexOf(edge) + ": " + edge.toString()));
         group.getChildren().add(edgeNode);
      }

      return group;
   }

   public static Node generateMultilineMesh(List<? extends Point3DReadOnly> multiline, boolean close, Color color, double width)
   {
      Group group = new Group();

      if (multiline == null)
         return group;

      if (multiline.size() < 2)
         return group;

      for (int i = 1; i < multiline.size(); i++)
      {
         Point3DReadOnly start = multiline.get(i - 1);
         Point3DReadOnly end = multiline.get(i);
         MeshDataHolder line = MeshDataGenerator.Line(start, end, width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         group.getChildren().add(edgeNode);
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("start: " + EuclidCoreIOTools.getTuple3DString(start) + ", end: "
               + EuclidCoreIOTools.getTuple3DString(end)));
      }

      if (close)
      {
         Point3DReadOnly start = multiline.get(multiline.size() - 1);
         Point3DReadOnly end = multiline.get(0);
         MeshDataHolder line = MeshDataGenerator.Line(start, end, width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         group.getChildren().add(edgeNode);
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("start: " + EuclidCoreIOTools.getTuple3DString(start) + ", end: "
               + EuclidCoreIOTools.getTuple3DString(end)));
      }

      return group;
   }

   public static Color nextColor(Random random)
   {
      return Color.hsb(EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0), 0.9, 0.9);
   }

   public static void main(String[] args)
   {
      launch();
   }
}
