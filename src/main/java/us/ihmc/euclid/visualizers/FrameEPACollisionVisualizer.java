package us.ihmc.euclid.visualizers;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.epa.FrameExpandingPolytopeAlgorithm;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class FrameEPACollisionVisualizer extends Application
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameExpandingPolytopeAlgorithm frameEPADetector = new FrameExpandingPolytopeAlgorithm();
   private final ExpandingPolytopeAlgorithm framelessEPADetector = new ExpandingPolytopeAlgorithm();

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
                                                                                                new RigidBodyTransform(0.9979046324088627000,
                                                                                                                       -0.0633693092670992700,
                                                                                                                       -0.0130642741835739760,
                                                                                                                       0.7443977771656639000,
                                                                                                                       0.0642935947651630800,
                                                                                                                       0.9938321600326341000,
                                                                                                                       0.0903546974818785800,
                                                                                                                       0.5727110955631666000,
                                                                                                                       0.0072579810626555420,
                                                                                                                       -0.0910053203273277100,
                                                                                                                       0.9958239570240388000,
                                                                                                                       -0.7135602367629830000));
      ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameB",
                                                                                                worldFrame,
                                                                                                new RigidBodyTransform(0.8181030810359122000,
                                                                                                                       -0.4435472606074187000,
                                                                                                                       -0.3660234642850132000,
                                                                                                                       -0.8255052417260503000,
                                                                                                                       0.5443712066093924000,
                                                                                                                       0.8024906551443900000,
                                                                                                                       0.2442718522887194500,
                                                                                                                       0.4188216382384691300,
                                                                                                                       0.1853842987261379000,
                                                                                                                       -0.3990921898679330400,
                                                                                                                       0.8979744349213044000,
                                                                                                                       -0.4012192076505227000));

      FrameCapsule3D shapeA = new FrameCapsule3D(frameA,
                                                 new Point3D(0.0740430166234076200, 0.3979202226247427000, 0.2158481198537354200),
                                                 new Vector3D(0.7989355029115155000, 0.1156035416199616400, -0.5902015616316562000),
                                                 0.2612493918222833500,
                                                 0.2733605158455267000);
      RotationMatrix orientationB = new RotationMatrix();
      orientationB.setUnsafe(0.1898011460775145300,
                             0.8468316901642968000,
                             0.4968416382320852700,
                             -0.7467763481264588000,
                             0.4530451734407276000,
                             -0.4869036421110147000,
                             -0.6374171403614528000,
                             -0.2786147148941373000,
                             0.7183823702025869000);
      FrameRamp3D shapeB = new FrameRamp3D(frameB,
                                           new Point3D(-0.8608149443370450000, -0.9512759705732754000, 0.3626654411902035000),
                                           orientationB,
                                           0.4637792371183097000,
                                           0.4203001274368498600,
                                           0.868342026938451300);

      EuclidFrameShape3DCollisionResult frameResult = new EuclidFrameShape3DCollisionResult();
      frameEPADetector.evaluateCollision(shapeA, shapeB, frameResult);
      frameResult.getPointOnA().changeFrame(worldFrame);
      frameResult.getPointOnB().changeFrame(worldFrame);

      FrameShape3DBasics shapeAInDetectorFrame = shapeA.copy();
      FrameShape3DBasics shapeBInDetectorFrame = shapeB.copy();
      shapeAInDetectorFrame.changeFrame(frameEPADetector.getDetectorFrame());
      shapeBInDetectorFrame.changeFrame(frameEPADetector.getDetectorFrame());
      EuclidShape3DCollisionResult framelessResult = new EuclidShape3DCollisionResult();
      framelessEPADetector.evaluateCollision(shapeAInDetectorFrame, shapeBInDetectorFrame, framelessResult);
      frameEPADetector.getDetectorFrame().transformFromThisToDesiredFrame(worldFrame, framelessResult);

      System.out.println("Frame:\n" + frameResult);
      System.out.println("Frameless:\n" + framelessResult);

      System.out.println(frameResult.getPointOnA().distance(frameResult.getPointOnB()));
      System.out.println(framelessResult.getPointOnA().distance(framelessResult.getPointOnB()));

      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(framelessResult.getPointOnA(), Color.ORANGE, 0.01));
      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(framelessResult.getPointOnB(), Color.ORANGERED, 0.01));

      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(frameResult.getPointOnA(), Color.AQUAMARINE, 0.01));
      view3dFactory.addNodeToView(Shape3DMeshFactories.togeneratePointMesh(frameResult.getPointOnB(), Color.CADETBLUE, 0.01));

      view3dFactory.addNodeToView(Shape3DMeshFactories.toFrameShape3DMesh(shapeA, Color.AQUAMARINE.deriveColor(0, 1, 1, 0.5)));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toFrameShape3DMesh(shapeB, Color.CORNFLOWERBLUE.deriveColor(0, 1, 1, 0.5)));

      EPAConvexPolytope3D polytope = new EPAConvexPolytope3D(frameEPADetector.getClosestFace());
      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsMesh(polytope.getFaces()));

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
      launch(args);
   }
}
