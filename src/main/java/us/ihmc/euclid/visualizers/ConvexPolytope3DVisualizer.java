package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import com.sun.corba.se.impl.protocol.giopmsgheaders.Message;

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
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3DTroublesomeDatasetLibrary.*;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;

public class ConvexPolytope3DVisualizer extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(600, 400);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.01, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.001);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, 1.0, Color.WHEAT);

      ConvexPolytope3DTroublesomeDataset dataset = new ConvexPolytope3DTroublesomeDataset_20190302_160115();
      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(dataset.getPointsBeforeIssue()), dataset.getConstructionEpsilon());
      convexPolytope3D.addVertex(dataset.getTroublesomePoint());
      view3dFactory.addNodeToView(generateFace3DsMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(generateFace3DsNormalMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(generatePointMesh(dataset.getTroublesomePoint(), Color.AQUAMARINE, 0.005));

//      List<Face3D> visibleFaces = new ArrayList<>();
//      List<Face3D> inPlaneFaces = new ArrayList<>();
//      Collection<HalfEdge3DReadOnly> silhouetteEdges = EuclidPolytopeTools.computeSilhouette(convexPolytope3D.getFaces(), dataset.getTroublesomePoint(),
//                                                                                             dataset.getConstructionEpsilon(), visibleFaces, inPlaneFaces);
//      view3dFactory.addNodeToView(generateHalfEdge3DsMesh(silhouetteEdges, Color.YELLOW));
//      view3dFactory.addNodeToView(generateFace3DsMesh(inPlaneFaces, Color.DARKRED));
//      view3dFactory.addNodeToView(generateFace3DsMesh(visibleFaces, Color.GREEN));

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
      double hueDelta = 360.0 / faces.size();
      double hue = 0.0;

      for (F face : faces)
      {
         List<Point3D> cwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);

         MeshView outsideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(ccwFaceVertices)));
         outsideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println(face.toString()));
         outsideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 0.9)));
         group.getChildren().add(outsideFaceNode);

         MeshView insideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(cwFaceVertices)));
         insideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println(face.toString()));
         insideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 0.7)));
         group.getChildren().add(insideFaceNode);
         hue += hueDelta;
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
      double hueDelta = 360.0 / faces.size();
      double hue = 0.0;
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(1024));

      for (Face3DReadOnly face : faces)
      {
         double scale = Math.max(0.03, face.getEdges().stream().mapToDouble(HalfEdge3DReadOnly::length).max().getAsDouble());
         double height = 0.010 * scale;
         double radius = 0.005 * scale;
         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(face.getNormal());
         meshBuilder.addCone(height, radius, face.getCentroid(), orientation, Color.hsb(hue, 0.9, 0.7));
         hue += hueDelta;
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
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
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Line(edge.getOrigin(), edge.getDestination(), 0.002)));
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

   public static void main(String[] args)
   {
      launch();
   }
}
