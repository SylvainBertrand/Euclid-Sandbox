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
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
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
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(600, 400);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.005, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.0001);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, -1.0, Color.WHEAT);

      ConvexPolytope3DTroublesomeDataset dataset = new ConvexPolytope3DTroublesomeDataset_20190305_230259();

      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(dataset.getPointsBeforeIssue()),
                                                               dataset.getConstructionEpsilon());
      //      convexPolytope3D.addVertex(dataset.getTroublesomePoint());
      view3dFactory.addNodeToView(generateFace3DsMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(generateFace3DsNormalMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(generatePointMesh(dataset.getTroublesomePoint(), Color.AQUAMARINE, 0.1));
      Vector3D supportDirection = new Vector3D();
      supportDirection.sub(dataset.getTroublesomePoint(), convexPolytope3D.getCentroid());

//      view3dFactory.addNodeToView(generateFace3DsMesh(Collections.singletonList(convexPolytope3D.getClosestFace(dataset.getTroublesomePoint())),
//                                                      Color.DARKRED));
            view3dFactory.addNodeToView(generateFace3DsMesh(Collections.singletonList(findClosestFace(convexPolytope3D, dataset.getTroublesomePoint())),
                                                            Color.GREEN));

      System.out.println(findClosestFace(convexPolytope3D, dataset.getTroublesomePoint()).isPointDirectlyAboveOrBelow(dataset.getTroublesomePoint()));
      System.out.println(convexPolytope3D.getClosestFace(dataset.getTroublesomePoint()).isPointDirectlyAboveOrBelow(dataset.getTroublesomePoint()));

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
         outsideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0)));
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

   public static class ConvexPolytope3DTroublesomeDataset_20190305_230259 extends ConvexPolytope3DTroublesomeDataset
   {
      public ConvexPolytope3DTroublesomeDataset_20190305_230259()
      {
         pointsBeforeIssue.add(new Point3D(0.42916866109613850000, 0.35206728716550995000, 0.24198843507781653000));
         pointsBeforeIssue.add(new Point3D(1.46433067655042670000, -5.26377783593307850000, 2.44618277677921370000));
         pointsBeforeIssue.add(new Point3D(0.61830334684053940000, 2.66349260406482370000, 6.38060838183427600000));
         pointsBeforeIssue.add(new Point3D(1.37824795980955270000, -4.98401625119844650000, 8.19666411831393300000));
         pointsBeforeIssue.add(new Point3D(0.39619803279994414000, 0.44391838094703484000, 9.72098574072407100000));
         pointsBeforeIssue.add(new Point3D(-0.26336361752563420000, -5.16126006270603450000, 8.86228850474557000000));
         pointsBeforeIssue.add(new Point3D(-3.12070099267314500000, 2.90520685186159880000, 6.39474568510551040000));
         pointsBeforeIssue.add(new Point3D(-3.48157599866199000000, 2.93922795294413050000, 1.71252352249696440000));
         pointsBeforeIssue.add(new Point3D(-2.87951312292417330000, -6.90648497714716100000, 0.72833263983926160000));
         pointsBeforeIssue.add(new Point3D(0.13861121914660401000, -6.52318469303109300000, 6.97138243677902400000));
         pointsBeforeIssue.add(new Point3D(-6.51223087001668800000, 2.86354377464792700000, 4.77084171815430100000));
         pointsBeforeIssue.add(new Point3D(-6.31679849968808800000, 0.95853464225646600000, 8.52267478973442200000));
         pointsBeforeIssue.add(new Point3D(-7.64597685912712500000, -0.07575959233231178000, 6.32871693283292700000));
         pointsBeforeIssue.add(new Point3D(-5.77449537389847800000, 0.91519549893172590000, 9.38387291036445000000));
         pointsBeforeIssue.add(new Point3D(-1.93743382960031770000, 1.98238452138590930000, 8.32900725005948500000));
         pointsBeforeIssue.add(new Point3D(-6.11840079677843700000, -2.09781287024978360000, 0.07049430718593896000));
         pointsBeforeIssue.add(new Point3D(-7.75265488757376200000, -1.95895694857336100000, 0.54151360675960800000));
         pointsBeforeIssue.add(new Point3D(-7.08717097972498960000, -4.89751394464854800000, 1.59825847267978200000));
         pointsBeforeIssue.add(new Point3D(-8.27817599967944200000, -4.45189333962035000000, 5.86399813481871700000));
         pointsBeforeIssue.add(new Point3D(-2.44177070225528150000, 2.75592679803204000000, 0.63511959090570350000));
         pointsBeforeIssue.add(new Point3D(-8.32041956802220300000, -2.66176636678776560000, 5.61847328699008800000));
         pointsBeforeIssue.add(new Point3D(-4.97112015016016600000, -0.97990274519680120000, 9.73185501783019500000));
         pointsBeforeIssue.add(new Point3D(-6.39642541488284200000, -6.31532117691915100000, 8.92760802580350300000));
         pointsBeforeIssue.add(new Point3D(0.73108401443885280000, -1.79992017788094930000, 0.83140647918804780000));
         pointsBeforeIssue.add(new Point3D(-2.62537465739533450000, -4.87023023131946700000, 0.11512860469325048000));
         pointsBeforeIssue.add(new Point3D(-6.41605375410656500000, -1.69013847222851420000, 8.66544643548335700000));
         pointsBeforeIssue.add(new Point3D(-6.78540730466985400000, -5.84718005795835700000, 8.52618891230881100000));
         pointsBeforeIssue.add(new Point3D(-6.47344344882216000000, -6.74681158021361000000, 8.87479129834585300000));
         troublesomePoint.set(-7.06446074090651100000, -1.40670426625248900000, -1.87693321160715950000);
         constructionEpsilon = 1.0E-10;
      }
   }

   public static Face3DReadOnly findClosestFace(ConvexPolytope3DReadOnly convexPolytope3D, Point3DReadOnly point)
   {
      boolean isOutside = false;
      double maxNegativeDistance = Double.NEGATIVE_INFINITY;
      Face3DReadOnly closestFace = null;

      for (int faceIndex = 0; faceIndex < convexPolytope3D.getNumberOfFaces(); faceIndex++)
      {
         Face3DReadOnly face = convexPolytope3D.getFace(faceIndex);
         double signedDistanceToPlane = face.signedDistanceToPlane(point);

         if (signedDistanceToPlane < 0.0)
         {
            maxNegativeDistance = Math.max(maxNegativeDistance, signedDistanceToPlane);
            closestFace = face;
         }
         else
         {
            isOutside = true;
            closestFace = face;
            break;
         }
      }

      if (!isOutside)
         return closestFace;

      boolean goToNeighbor = false;
      boolean isQueryDirectlyAboveFace = true;
      Face3DReadOnly candidateFace = null;
      double distanceToClosestEdge = Double.POSITIVE_INFINITY;

      while (true)
      {
         goToNeighbor = false;
         isQueryDirectlyAboveFace = true;
         distanceToClosestEdge = Double.POSITIVE_INFINITY;

         for (int edgeIndex = 0; edgeIndex < closestFace.getNumberOfEdges(); edgeIndex++)
         {
            HalfEdge3DReadOnly edge = closestFace.getEdge(edgeIndex);
            boolean isEdgeVisible = closestFace.canObserverSeeEdge(point, edge);

            if (isEdgeVisible)
            { // Visit neighbor.
               isQueryDirectlyAboveFace = false;
               HalfEdge3DReadOnly neighborEdge = edge.getTwin();
               candidateFace = neighborEdge.getFace();
               boolean isNeighborEdgeVisible = candidateFace.canObserverSeeEdge(point, neighborEdge);

               if (!isNeighborEdgeVisible)
               {
                  closestFace = candidateFace;
                  goToNeighbor = true;
                  break;
               }
               else
               {
                  distanceToClosestEdge = Math.min(distanceToClosestEdge, neighborEdge.distance(point));
               }
            }

         }

         if (!goToNeighbor)
         {
            return closestFace;
         }
      }
   }

   public static void main(String[] args)
   {
      launch();
   }
}
