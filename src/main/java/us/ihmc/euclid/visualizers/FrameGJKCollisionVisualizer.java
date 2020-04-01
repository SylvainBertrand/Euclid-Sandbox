package us.ihmc.euclid.visualizers;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.gjk.FrameGilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameBox3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameRamp3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class FrameGJKCollisionVisualizer extends Application
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameGilbertJohnsonKeerthiCollisionDetector frameGJKDetector = new FrameGilbertJohnsonKeerthiCollisionDetector();
   private final GilbertJohnsonKeerthiCollisionDetector framelessGJKDetector = new GilbertJohnsonKeerthiCollisionDetector();

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

      ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameA",
                                                                                                worldFrame,
                                                                                                new RigidBodyTransform(0.266752199359763130000,
                                                                                                                       0.943854110092642300000,
                                                                                                                       -0.194891464661629800000,
                                                                                                                       0.110356424430389510000,
                                                                                                                       0.756945120948152800000,
                                                                                                                       -0.080009823889612490000,
                                                                                                                       0.648561879818679700000,
                                                                                                                       -0.042245380800103316000,
                                                                                                                       0.596554564151105600000,
                                                                                                                       -0.320527451152595200000,
                                                                                                                       -0.735788560014427700000,
                                                                                                                       -1.143964174244656000000));
      ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameB", worldFrame, new RigidBodyTransform());

      RotationMatrix orientationA = new RotationMatrix();
      orientationA.setUnsafe(0.347962419350583700000,
                             -0.763257890834793200000,
                             0.544389150147310900000,
                             0.550092212145841300000,
                             0.636431331143421000000,
                             0.540697437459720200000,
                             -0.759157897183572300000,
                             0.111321843397583550000,
                             0.641316407342257600000);
      FrameRamp3D shapeA = new FrameRamp3D(frameA,
                                           new Point3D(0.480594886854139400000, -0.595094351811325400000, -0.268769645145817000000),
                                           orientationA,
                                           0.383334226382238130000,
                                           0.198316076990102900000,
                                           0.817079807458062300000);
      RotationMatrix orientationB = new RotationMatrix();
      orientationB.setUnsafe(0.408196431800644600000,
                             0.334105731006358600000,
                             0.849558140196378100000,
                             -0.644161194106022400000,
                             0.764840077583117500000,
                             0.008718470654569932000,
                             -0.646863222847864100000,
                             -0.550811234663344500000,
                             0.527422937210278100000);
      FrameBox3D shapeB = new FrameBox3D(frameB,
                                         new Point3D(-0.697733308154290700000, 0.432362483157936900000, -0.277329711388412100000),
                                         orientationB,
                                         0.562226605854888500000,
                                         0.109060911353156880000,
                                         0.761961161606297300000);

      view3dFactory.addNodeToView(generateFrameRamp3DMesh(shapeA, Color.AQUAMARINE));
      view3dFactory.addNodeToView(generateFrameBox3DMesh(shapeB, Color.CORNFLOWERBLUE));

      EuclidFrameShape3DCollisionResult frameResult = frameGJKDetector.evaluateCollision(shapeA, shapeB);
      frameResult.getPointOnA().changeFrame(worldFrame);
      frameResult.getPointOnB().changeFrame(worldFrame);

      FrameShape3DBasics shapeAInDetectorFrame = shapeA.copy();
      FrameShape3DBasics shapeBInDetectorFrame = shapeB.copy();
      shapeAInDetectorFrame.changeFrame(frameGJKDetector.getDetectorFrame());
      shapeBInDetectorFrame.changeFrame(frameGJKDetector.getDetectorFrame());
      EuclidShape3DCollisionResult framelessResult = framelessGJKDetector.evaluateCollision(shapeAInDetectorFrame, shapeBInDetectorFrame);
      frameGJKDetector.getDetectorFrame().transformFromThisToDesiredFrame(worldFrame, framelessResult);

      System.out.println("Frame:\n" + frameResult);
      System.out.println("Frameless:\n" + framelessResult);

      System.out.println(frameResult.getPointOnA().distance(frameResult.getPointOnB()));
      System.out.println(framelessResult.getPointOnA().distance(framelessResult.getPointOnB()));
      

      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(framelessResult.getPointOnA(), Color.ORANGE, 0.01));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(framelessResult.getPointOnB(), Color.ORANGERED, 0.01));

      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(frameResult.getPointOnA(), Color.AQUAMARINE, 0.01));
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(frameResult.getPointOnB(), Color.CADETBLUE, 0.01));

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

   public static Node generateFrameRamp3DMesh(FrameRamp3DReadOnly ramp3D, Color color)
   {
      FixedFrameRamp3DBasics copy = ramp3D.copy();
      ramp3D.getReferenceFrame().transformFromThisToDesiredFrame(worldFrame, copy);
      return GJKCollisionVisualizer.generateRamp3DMesh(copy, color);
   }

   public static Node generateFrameBox3DMesh(FrameBox3DReadOnly ramp3D, Color color)
   {
      FixedFrameBox3DBasics copy = ramp3D.copy();
      ramp3D.getReferenceFrame().transformFromThisToDesiredFrame(worldFrame, copy);
      return GJKCollisionVisualizer.generateBox3DMesh(copy, color);
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
