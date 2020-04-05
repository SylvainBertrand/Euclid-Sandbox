package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.EuclidShapeCollisionTools;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.tuple3D.Point3D;
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

      PointShape3D shapeA = new PointShape3D(0.10918800395708050000, -0.50000462201989260000, 0.29714612703546717000);
      RotationMatrix orientation = new RotationMatrix();
      orientation.setUnsafe(0.98164214130999300000,
                            -0.18497632661290123000,
                            0.04650231173962113000,
                            0.17010111240946710000,
                            0.95933305891523400000,
                            0.22526804839879544000,
                            -0.08628046106386697000,
                            -0.21322251444239532000,
                            0.97318643710928740000);
      Ramp3D shapeB = new Ramp3D(new Point3D(-0.39950050686918190000, -0.54177410875284450000, 0.24049418965939195000),
                                 orientation,
                                 0.73542974157294870000,
                                 0.98399444333778800000,
                                 0.98154753108250430000);

      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
      EuclidShapeCollisionTools.evaluatePointShape3DRamp3DCollision(shapeA, shapeB, result);
      System.out.println("Expected:\n\t" + result);

      ExpandingPolytopeAlgorithm epaDetector = new ExpandingPolytopeAlgorithm();
      EuclidShape3DCollisionResult epaResult = epaDetector.evaluateCollision(shapeA, shapeB);
      System.out.println("EPA:\n\t" + epaResult);

      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(result.getPointOnA(), Color.ORANGE, 0.01));
      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(result.getPointOnB(), Color.ORANGERED, 0.01));

      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(epaResult.getPointOnA(), Color.AQUAMARINE, 0.01));
      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(epaResult.getPointOnB(), Color.CADETBLUE, 0.01));

      view3dFactory.addNodeToView(Shape3DMeshFactories.toShape3DMesh(shapeA, Color.AQUAMARINE.deriveColor(0, 1, 1, 0.5)));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toShape3DMesh(shapeB, Color.CORNFLOWERBLUE.deriveColor(0, 1, 1, 0.5)));

      EPAConvexPolytope3D polytope = new EPAConvexPolytope3D(epaDetector.getClosestFace());
      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsMesh(polytope.getFaces()));

      Point3D position = new Point3D(0.73542974157294870000, -0.49199722166889400000, 0.98154753108250430000);
      position.applyTransform(shapeB.getPose());
      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(position, Color.BLACK, 0.01));
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
         Vertex3D v0 = new Vertex3D(1.32560367365175560000, 0.61038957712858410000, -3.28296007438939960000);
         Vertex3D v1 = new Vertex3D(2.55176816189094070000, 1.56699832754669990000, -3.48118334549167850000);
         Vertex3D v2 = new Vertex3D(2.77889615125680670000, 0.16603661879546128000, -3.66813995672885040000);
         Vertex3D v3 = new Vertex3D(-0.14808308303696283000, 1.12315868400520810000, -3.43523385740898200000);
         Vertex3D v4 = new Vertex3D(0.07904490632890315000, -0.27780302474603070000, -3.62219046864615370000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)),
                              new Vector3D(0.22864807304482535000, -0.09225978343536065000, 0.96912754116955560000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v3), new HalfEdge3D(v3, v1)),
                              new Vector3D(-0.02101943597882897200, 0.22862187620776278000, 0.97328835451287220000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v3), new HalfEdge3D(v3, v4), new HalfEdge3D(v4, v2)),
                              new Vector3D(-0.03761571379618803400, 0.12618844000771007000, -0.99129286070465770000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v4), new HalfEdge3D(v4, v0)),
                              new Vector3D(0.09137388834406775000, -0.46463809644139054000, 0.88077366665006910000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v3, v0), new HalfEdge3D(v0, v4), new HalfEdge3D(v4, v3)),
                              new Vector3D(-0.15485115144541460000, -0.15530479727305150000, 0.97565441670706620000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
      }
   }

   public class PolytopeB extends ConvexPolytope3DTroublesomeDataset
   {
      public PolytopeB()
      {
         double constructionEpsilon = 1.0E-10;
         Vertex3D v0 = new Vertex3D(2.55062468789334450000, 1.56613553815613530000, -3.48118642412273400000);
         Vertex3D v1 = new Vertex3D(0.71882413777046890000, 1.42727286629596820000, -3.36201950370981100000);
         Vertex3D v2 = new Vertex3D(1.67822447541872100000, 0.15334692939893219000, -2.68625053474080880000);
         Vertex3D v3 = new Vertex3D(1.08023626819140600000, 0.94543695225306800000, -2.56376176092052740000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)),
                              new Vector3D(-0.02060300047136870400, -0.48057746030225850000, -0.87671022636958450000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v3), new HalfEdge3D(v3, v1)),
                              new Vector3D(-0.03015681249783523600, 0.84963815794487700000, 0.52650315025079060000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v3), new HalfEdge3D(v3, v2)),
                              new Vector3D(-0.79825774278928400000, -0.60231217575778940000, -0.00214918842245640260),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v0)),
                              new Vector3D(0.45307014488918607000, 0.20799219403771907000, 0.86687178465414790000),
                              constructionEpsilon));
         convexPolytope3D = new ConvexPolytope3D(faces, constructionEpsilon);
      }
   }

   public class PolytopeBTranslated extends ConvexPolytope3DTroublesomeDataset
   {
      public PolytopeBTranslated()
      {
         double constructionEpsilon = 1.0E-10;
         Vertex3D v0 = new Vertex3D(2.55058470955042040000, 1.56626965244557770000, -3.48223997972286450000);
         Vertex3D v1 = new Vertex3D(0.71878415942754480000, 1.42740698058541060000, -3.36307305930994140000);
         Vertex3D v2 = new Vertex3D(1.67818449707579690000, 0.15348104368837454000, -2.68730409034093930000);
         Vertex3D v3 = new Vertex3D(1.08019628984848180000, 0.94557106654251030000, -2.56481531652065800000);
         List<Face3D> faces = new ArrayList<>();
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v1), new HalfEdge3D(v1, v2), new HalfEdge3D(v2, v0)),
                              new Vector3D(-0.02060300047136871000, -0.48057746030225845000, -0.87671022636958450000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v1, v0), new HalfEdge3D(v0, v3), new HalfEdge3D(v3, v1)),
                              new Vector3D(-0.03015681249783523000, 0.84963815794487700000, 0.52650315025079090000),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v2, v1), new HalfEdge3D(v1, v3), new HalfEdge3D(v3, v2)),
                              new Vector3D(-0.79825774278928400000, -0.60231217575778950000, -0.00214918842245653140),
                              constructionEpsilon));
         faces.add(new Face3D(Arrays.asList(new HalfEdge3D(v0, v2), new HalfEdge3D(v2, v3), new HalfEdge3D(v3, v0)),
                              new Vector3D(0.45307014488918610000, 0.20799219403771907000, 0.86687178465414800000),
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
