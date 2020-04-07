package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class EPACollisionVisualizer extends Application
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

      ConvexPolytope3D shapeA = new PolytopeB().getConvexPolytope3D();
      ConvexPolytope3D shapeB = new PolytopeA().getConvexPolytope3D();

      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      //      EuclidShapeCollisionTools.evaluatePointShape3DRamp3DCollision(shapeA, shapeB, result);
      System.out.println("Expected:\n\t" + result);

      ExpandingPolytopeAlgorithm epaDetector = new ExpandingPolytopeAlgorithm();
      EuclidShape3DCollisionResult epaResult = epaDetector.evaluateCollision((SupportingVertexHolder) shapeB, (SupportingVertexHolder) shapeA);
      System.out.println("EPA:\n\t" + epaResult);

      view3dFactory.addNodeToView(Shape3DMeshFactories.toPointMesh(result.getPointOnA(), Color.ORANGE, 0.01));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toPointMesh(result.getPointOnB(), Color.ORANGERED, 0.01));

      view3dFactory.addNodeToView(Shape3DMeshFactories.toPointMesh(epaResult.getPointOnA(), Color.AQUAMARINE, 0.01));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toPointMesh(epaResult.getPointOnB(), Color.CADETBLUE, 0.01));

      view3dFactory.addNodeToView(Shape3DMeshFactories.toShape3DMesh(shapeA, Color.AQUAMARINE.deriveColor(0, 1, 1, 0.5)));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toShape3DMesh(shapeB, Color.CORNFLOWERBLUE.deriveColor(0, 1, 1, 0.5)));

      EPAConvexPolytope3D polytope = new EPAConvexPolytope3D(epaDetector.getClosestFace());
      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsMesh(polytope.getFaces()));

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
   }

   public class PolytopeA extends ConvexPolytope3DTroublesomeDataset
   {
      public PolytopeA()
      {
         double constructionEpsilon = 1.0E-10;
         Vertex3D v0 = new Vertex3D(-0.19999999999999996000, -0.70000000000000000000, -0.57000000000000000000);
         Vertex3D v1 = new Vertex3D(-0.19999999999999996000, 0.50000000000000000000, -0.57000000000000000000);
         Vertex3D v2 = new Vertex3D(-1.20000000000000000000, 0.50000000000000000000, -0.57000000000000000000);
         Vertex3D v3 = new Vertex3D(-1.20000000000000000000, -0.70000000000000000000, -0.57000000000000000000);
         Vertex3D v4 = new Vertex3D(-1.20000000000000000000, 0.50000000000000000000, 0.83000000000000000000);
         Vertex3D v5 = new Vertex3D(-1.20000000000000000000, -0.70000000000000000000, 0.83000000000000000000);
         Vertex3D v6 = new Vertex3D(-0.19999999999999996000, -0.70000000000000000000, 0.83000000000000000000);
         Vertex3D v7 = new Vertex3D(-0.19999999999999996000, 0.50000000000000000000, 0.83000000000000000000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v0)),
                              new Vector3D(-0.00000000000000000000, -0.00000000000000000000, -1.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v2), new HalfEdge3D(v2, v4), new HalfEdge3D(v4, v5), new HalfEdge3D(v5, v3)),
                              new Vector3D(-1.00000000000000000000, -0.00000000000000000000, -0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v3), new HalfEdge3D(v3, v5), new HalfEdge3D(v5, v6), new HalfEdge3D(v6, v0)),
                              new Vector3D(-0.00000000000000000000, -1.00000000000000000000, -0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v6), new HalfEdge3D(v6, v7), new HalfEdge3D(v7, v1)),
                              new Vector3D(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v7), new HalfEdge3D(v7, v4), new HalfEdge3D(v4, v2)),
                              new Vector3D(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v6, v5), new HalfEdge3D(v5, v4), new HalfEdge3D(v4, v7), new HalfEdge3D(v7, v6)),
                              new Vector3D(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
      }
   }

   public class PolytopeB extends ConvexPolytope3DTroublesomeDataset
   {
      public PolytopeB()
      {
         double constructionEpsilon = 1.0E-10;
         Vertex3D v0 = new Vertex3D(0.09999999999999998000, 0.25000000000000000000, -0.39999999999999997000);
         Vertex3D v1 = new Vertex3D(0.09999999999999998000, 1.45000000000000000000, -0.39999999999999997000);
         Vertex3D v2 = new Vertex3D(-0.90000000000000000000, 1.45000000000000000000, -0.39999999999999997000);
         Vertex3D v3 = new Vertex3D(-0.90000000000000000000, 0.25000000000000000000, -0.39999999999999997000);
         Vertex3D v4 = new Vertex3D(-0.90000000000000000000, 1.45000000000000000000, 1.00000000000000000000);
         Vertex3D v5 = new Vertex3D(-0.90000000000000000000, 0.25000000000000000000, 1.00000000000000000000);
         Vertex3D v6 = new Vertex3D(0.09999999999999998000, 0.25000000000000000000, 1.00000000000000000000);
         Vertex3D v7 = new Vertex3D(0.09999999999999998000, 1.45000000000000000000, 1.00000000000000000000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v0)),
                              new Vector3D(-0.00000000000000000000, -0.00000000000000000000, -1.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v2), new HalfEdge3D(v2, v4), new HalfEdge3D(v4, v5), new HalfEdge3D(v5, v3)),
                              new Vector3D(-1.00000000000000000000, -0.00000000000000000000, -0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v3), new HalfEdge3D(v3, v5), new HalfEdge3D(v5, v6), new HalfEdge3D(v6, v0)),
                              new Vector3D(-0.00000000000000000000, -1.00000000000000000000, -0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v6), new HalfEdge3D(v6, v7), new HalfEdge3D(v7, v1)),
                              new Vector3D(1.00000000000000000000, 0.00000000000000000000, 0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v7), new HalfEdge3D(v7, v4), new HalfEdge3D(v4, v2)),
                              new Vector3D(0.00000000000000000000, 1.00000000000000000000, 0.00000000000000000000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v6, v5), new HalfEdge3D(v5, v4), new HalfEdge3D(v4, v7), new HalfEdge3D(v7, v6)),
                              new Vector3D(0.00000000000000000000, 0.00000000000000000000, 1.00000000000000000000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
      }
   }

   @Override
   public void stop()
   {
      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
