package us.ihmc.euclid.visualizers;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePointShape3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.epa.FrameExpandingPolytopeAlgorithm;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
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

      ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, new RigidBodyTransform());
      ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameB",
                                                                                                worldFrame,
                                                                                                new RigidBodyTransform(0.657583346332029300000,
                                                                                                                       -0.752913560191715700000,
                                                                                                                       -0.026557739101927430000,
                                                                                                                       0.404602104151486230000,
                                                                                                                       -0.682471756172704800000,
                                                                                                                       -0.580389493433878700000,
                                                                                                                       -0.444275070128979700000,
                                                                                                                       -0.823608500116551700000,
                                                                                                                       0.319086892011117560000,
                                                                                                                       0.310272794152180400000,
                                                                                                                       -0.895496704938485000000,
                                                                                                                       -0.048811735620764296000));

      FramePointShape3D shapeA = new FramePointShape3D(frameA, -0.187058996268268720000, -0.665776855716039500000, -0.002390707611566123000);
      RotationMatrix orientationB = new RotationMatrix();
      orientationB.setUnsafe(0.341324525827703100000,
                             -0.847591639295407500000,
                             0.406307741822646060000,
                             -0.881657805727667600000,
                             -0.138848486059230500000,
                             0.450999569310808150000,
                             -0.325848249447451400000,
                             -0.512161606249027500000,
                             -0.794678178520366600000);
      FrameBox3D shapeB = new FrameBox3D(frameB,
                                         new Point3D(-0.462885653261269740000, 0.416768783152731140000, -0.067700988147070480000),
                                         orientationB,
                                         0.574933452856738200000,
                                         0.531349973266704300000,
                                         0.797236491604658500000);

      EuclidFrameShape3DCollisionResult frameResult = new EuclidFrameShape3DCollisionResult();
      frameEPADetector.evaluateCollision(shapeA, shapeB, frameResult);
      frameResult.getPointOnA().changeFrame(worldFrame);
      frameResult.getPointOnB().changeFrame(worldFrame);

      FrameShape3DBasics shapeAInDetectorFrame = shapeA.copy();
      FrameShape3DBasics shapeBInDetectorFrame = shapeB.copy();
      shapeAInDetectorFrame.changeFrame(frameEPADetector.getDetectorFrame());
      shapeBInDetectorFrame.changeFrame(frameEPADetector.getDetectorFrame());
      EuclidShape3DCollisionResult framelessResult = new EuclidShape3DCollisionResult();
      //      framelessEPADetector.evaluateCollision(shapeAInDetectorFrame, shapeBInDetectorFrame, framelessResult);
      frameEPADetector.getDetectorFrame().transformFromThisToDesiredFrame(worldFrame, framelessResult);

      System.out.println("Frame:\n" + frameResult);
      System.out.println("Frameless:\n" + framelessResult);

      System.out.println(frameResult.getPointOnA().distance(frameResult.getPointOnB()));
      System.out.println(framelessResult.getPointOnA().distance(framelessResult.getPointOnB()));

      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(framelessResult.getPointOnA(), Color.ORANGE, 0.01));
      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(framelessResult.getPointOnB(), Color.ORANGERED, 0.01));
      //
      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(frameResult.getPointOnA(), Color.AQUAMARINE, 0.01));
      //      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generatePointMesh(frameResult.getPointOnB(), Color.CADETBLUE, 0.01));
      //
      //      view3dFactory.addNodeToView(generateFrameShape3DMesh(shapeA, Color.AQUAMARINE.deriveColor(0, 1, 1, 0.5)));
      //      view3dFactory.addNodeToView(generateFrameShape3DMesh(shapeB, Color.CORNFLOWERBLUE.deriveColor(0, 1, 1, 0.5)));

      EPAConvexPolytope3D polytope = new EPAConvexPolytope3D(frameEPADetector.getClosestFace());
      view3dFactory.addNodeToView(ConvexPolytope3DVisualizer.generateFace3DsMesh(polytope.getFaces()));

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

   public static Node generateFrameShape3DMesh(FrameShape3DReadOnly shape3D, Color color)
   {
      FixedFrameShape3DBasics copy = shape3D.copy();
      shape3D.getReferenceFrame().transformFromThisToDesiredFrame(worldFrame, copy);

      if (shape3D instanceof FrameBox3DReadOnly)
         return GJKCollisionVisualizer.generateBox3DMesh((Box3DReadOnly) copy, color);
      if (shape3D instanceof FrameRamp3DReadOnly)
         return GJKCollisionVisualizer.generateRamp3DMesh((Ramp3DReadOnly) copy, color);
      if (shape3D instanceof FramePointShape3DReadOnly)
         return GJKCollisionVisualizer.generatePointShape3DMesh((PointShape3DReadOnly) copy, color);
      if (shape3D instanceof FrameEllipsoid3DReadOnly)
         return GJKCollisionVisualizer.generateEllipsoid3DMesh((Ellipsoid3DReadOnly) copy, color);
      throw new UnsupportedOperationException("Unsupported shape " + shape3D);
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
