package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.HalfEdge3D;
import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.shape.convexPolytope.tools.ConvexPolytope3DTroublesomeDataset;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

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

      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsMesh(convexPolytope3D.getFaces()));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsNormalMesh(convexPolytope3D.getFaces()));

      view3dFactory.addNodeToView(Shape3DMeshFactories.toMultilineMesh(Arrays.asList(new Point3D(0.4789278257837637000, -1.0709997264316717000, -0.3921167138841577000),
                                                                      new Point3D(-0.5706011857203901000, -1.5817801098497340000, -0.3254900162247995600),
                                                                      new Point3D(-0.1226494165834033400, 0.4219890315821706000, 0.1456226472088930600),
                                                                      new Point3D(0.1030819243869309800, -0.3550176957114557000, -0.1224997543555419500)),
                                                        true, Color.ORANGE, 0.005));

      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(troublesomePoint, Color.BLACK, 0.1));

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

   public static void main(String[] args)
   {
      launch();
   }
}
