package us.ihmc.euclid.visualizers;

import java.text.DecimalFormat;
import java.text.ParseException;
import java.util.List;
import java.util.Random;
import java.util.function.DoubleConsumer;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.transform.Affine;
import javafx.stage.Stage;
import javafx.util.StringConverter;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.gjk.FrameGilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.visualizers.Shape3DMeshFactories.UVMeshType;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.scenes.View3DFactory.SceneType;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class STPGJKAnimation extends Application
{
   @FXML
   private ToggleButton enableGJKButton, runButton, rewindButton;
   @FXML
   private Spinner<Double> angularVelocityAXSpinner, angularVelocityAYSpinner, angularVelocityAZSpinner;
   @FXML
   private Spinner<Double> angularVelocityBXSpinner, angularVelocityBYSpinner, angularVelocityBZSpinner;
   @FXML
   private Spinner<Double> positionAXSpinner, positionAYSpinner, positionAZSpinner;
   @FXML
   private Spinner<Double> positionBXSpinner, positionBYSpinner, positionBZSpinner;
   @FXML
   private Spinner<Double> yawASpinner, pitchASpinner, rollASpinner;
   @FXML
   private Spinner<Double> yawBSpinner, pitchBSpinner, rollBSpinner;

   private final DecimalFormat spinnerFormatter = new DecimalFormat("#.####");

   private boolean disablePoseSpinnerListeners = false;

   private final Vector3D angularVelocityA = new Vector3D();
   private final Vector3D angularVelocityB = new Vector3D();
   private final Vector3D rotationVectorA = new Vector3D();
   private final Vector3D rotationVectorB = new Vector3D();
   private final RotationMatrix rotationMatrixA = new RotationMatrix();
   private final RotationMatrix rotationMatrixB = new RotationMatrix();
   private final FrameVector3D stpInitialSupportDirection = new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.X);
   private final FrameVector3D initialSupportDirection = new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.X);

   private final RigidBodyTransform transformA = new RigidBodyTransform(new Quaternion(), new Vector3D(-2.0, 0.0, 0.0));
   private final RigidBodyTransform transformB = new RigidBodyTransform(new Quaternion(), new Vector3D(2.0, 0.0, 0.0));

   private final ReferenceFrame frameA = new ReferenceFrame("frameA", ReferenceFrame.getWorldFrame())
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(transformA);
      }
   };

   private final ReferenceFrame frameB = new ReferenceFrame("frameB", ReferenceFrame.getWorldFrame())
   {
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(transformB);
      }
   };

   @SuppressWarnings("unchecked")
   public void initialize()
   {
      double angularVelocityStep = 0.0125;
      angularVelocityAXSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityA.getX(), angularVelocityStep));
      angularVelocityAYSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityA.getY(), angularVelocityStep));
      angularVelocityAZSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityA.getZ(), angularVelocityStep));

      angularVelocityBXSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityB.getX(), angularVelocityStep));
      angularVelocityBYSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityB.getY(), angularVelocityStep));
      angularVelocityBZSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityB.getZ(), angularVelocityStep));

      angularVelocityAXSpinner.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityA.setX(newValue));
      angularVelocityAYSpinner.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityA.setY(newValue));
      angularVelocityAZSpinner.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityA.setZ(newValue));

      angularVelocityBXSpinner.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityB.setX(newValue));
      angularVelocityBYSpinner.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityB.setY(newValue));
      angularVelocityBZSpinner.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityB.setZ(newValue));

      positionAXSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, transformA.getTranslationX(), 0.05));
      positionAYSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, transformA.getTranslationY(), 0.05));
      positionAZSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, transformA.getTranslationZ(), 0.05));

      positionBXSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, transformB.getTranslationX(), 0.05));
      positionBYSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, transformB.getTranslationY(), 0.05));
      positionBZSpinner.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, transformB.getTranslationZ(), 0.05));

      Vector3DBasics translationA = transformA.getTranslation();
      positionAXSpinner.getValueFactory().valueProperty().addListener(poseListener(translationA::setX));
      positionAYSpinner.getValueFactory().valueProperty().addListener(poseListener(translationA::setY));
      positionAZSpinner.getValueFactory().valueProperty().addListener(poseListener(translationA::setZ));

      Vector3DBasics translationB = transformB.getTranslation();
      positionBXSpinner.getValueFactory().valueProperty().addListener(poseListener(translationB::setX));
      positionBYSpinner.getValueFactory().valueProperty().addListener(poseListener(translationB::setY));
      positionBZSpinner.getValueFactory().valueProperty().addListener(poseListener(translationB::setZ));

      double yawSpan = Math.PI;
      double pitchSpan = Math.PI / 2.0;
      double rollSpan = Math.PI;
      yawASpinner.setValueFactory(new DoubleSpinnerValueFactory(-yawSpan, yawSpan, transformA.getRotation().getYaw(), 0.025));
      pitchASpinner.setValueFactory(new DoubleSpinnerValueFactory(-pitchSpan, pitchSpan, transformA.getRotation().getPitch(), 0.025));
      rollASpinner.setValueFactory(new DoubleSpinnerValueFactory(-rollSpan, rollSpan, transformA.getRotation().getRoll(), 0.025));
      yawASpinner.getValueFactory().setWrapAround(true);
      pitchASpinner.getValueFactory().setWrapAround(true);
      rollASpinner.getValueFactory().setWrapAround(true);

      yawBSpinner.setValueFactory(new DoubleSpinnerValueFactory(-yawSpan, yawSpan, transformB.getRotation().getYaw(), 0.025));
      pitchBSpinner.setValueFactory(new DoubleSpinnerValueFactory(-pitchSpan, pitchSpan, transformB.getRotation().getPitch(), 0.025));
      rollBSpinner.setValueFactory(new DoubleSpinnerValueFactory(-rollSpan, rollSpan, transformB.getRotation().getRoll(), 0.025));
      yawBSpinner.getValueFactory().setWrapAround(true);
      pitchBSpinner.getValueFactory().setWrapAround(true);
      rollBSpinner.getValueFactory().setWrapAround(true);

      RotationMatrixBasics rotationA = transformA.getRotation();
      yawASpinner.getValueFactory().valueProperty().addListener(poseListener(yaw -> rotationA.setYawPitchRoll(yaw, rotationA.getPitch(), rotationA.getRoll())));
      pitchASpinner.getValueFactory().valueProperty()
                   .addListener(poseListener(pitch -> rotationA.setYawPitchRoll(rotationA.getYaw(), pitch, rotationA.getRoll())));
      rollASpinner.getValueFactory().valueProperty()
                  .addListener(poseListener(roll -> rotationA.setYawPitchRoll(rotationA.getYaw(), rotationA.getPitch(), roll)));

      RotationMatrixBasics rotationB = transformB.getRotation();
      yawBSpinner.getValueFactory().valueProperty().addListener(poseListener(yaw -> rotationB.setYawPitchRoll(yaw, rotationB.getPitch(), rotationB.getRoll())));
      pitchBSpinner.getValueFactory().valueProperty()
                   .addListener(poseListener(pitch -> rotationB.setYawPitchRoll(rotationB.getYaw(), pitch, rotationB.getRoll())));
      rollBSpinner.getValueFactory().valueProperty()
                  .addListener(poseListener(roll -> rotationB.setYawPitchRoll(rotationB.getYaw(), rotationB.getPitch(), roll)));

      confingureSpinners(angularVelocityAXSpinner,
                         angularVelocityAYSpinner,
                         angularVelocityAZSpinner,
                         angularVelocityBXSpinner,
                         angularVelocityBYSpinner,
                         angularVelocityBZSpinner,
                         positionAXSpinner,
                         positionAYSpinner,
                         positionAZSpinner,
                         positionBXSpinner,
                         positionBYSpinner,
                         positionBZSpinner,
                         yawASpinner,
                         pitchASpinner,
                         rollASpinner,
                         yawBSpinner,
                         pitchBSpinner,
                         rollBSpinner);
      frameA.update();
      frameB.update();
   }

   private void confingureSpinners(@SuppressWarnings("unchecked") Spinner<Double>... spinners)
   {
      for (Spinner<Double> spinner : spinners)
      {
         spinner.setEditable(true);

         spinner.getValueFactory().setConverter(new StringConverter<Double>()
         {
            @Override
            public String toString(Double value)
            {
               // If the specified value is null, return a zero-length String
               if (value == null)
                  return "";

               return spinnerFormatter.format(value);
            }

            @Override
            public Double fromString(String value)
            {
               try
               {
                  // If the specified value is null or zero-length, return null
                  if (value == null)
                     return null;

                  value = value.trim();

                  if (value.length() < 1)
                     return null;

                  // Perform the requested parsing
                  return spinnerFormatter.parse(value).doubleValue();
               }
               catch (ParseException ex)
               {
                  throw new RuntimeException(ex);
               }
            }
         });
      }
   }

   private ChangeListener<Double> poseListener(DoubleConsumer consumer)
   {
      return (observable, oldValue, newValue) ->
      {
         if (!disablePoseSpinnerListeners)
            consumer.accept(newValue);
      };
   }

   @FXML
   public void reset()
   {
      transformA.getRotation().setToZero();
      transformB.getRotation().setToZero();
      frameA.update();
      frameB.update();
      updatePoseSpinners();
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader loader = new FXMLLoader(getClass().getClassLoader().getResource("fxml/STPGJKAnimationWindow.fxml"));
      loader.setController(this);
      BorderPane mainPane = loader.load();

      View3DFactory view3dFactory = new View3DFactory(600, 400, true, SceneAntialiasing.BALANCED, SceneType.SUB_SCENE);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.001, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.1);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, -1.0, Color.WHEAT);

      PointLight light = new PointLight(Color.WHEAT);
      light.getTransforms().addAll(view3dFactory.getSubScene().getCamera().getTransforms());
      view3dFactory.addNodeToView(light);
      int resolution = 150;

      Random random = new Random(341);

      Sphere pointOnANode = new Sphere(0.01);
      pointOnANode.setMaterial(new PhongMaterial(Color.INDIGO));
      view3dFactory.addNodeToView(pointOnANode);
      Sphere stpPointOnANode = new Sphere(0.01);
      stpPointOnANode.setMaterial(new PhongMaterial(Color.GREENYELLOW));
      view3dFactory.addNodeToView(stpPointOnANode);

      Sphere pointOnBNode = new Sphere(0.01);
      pointOnBNode.setMaterial(new PhongMaterial(Color.INDIGO));
      view3dFactory.addNodeToView(pointOnBNode);
      Sphere stpPointOnBNode = new Sphere(0.01);
      stpPointOnBNode.setMaterial(new PhongMaterial(Color.GREENYELLOW));
      view3dFactory.addNodeToView(stpPointOnBNode);

      FrameBox3D shapeA = EuclidFrameShapeRandomTools.nextFrameBox3D(random, frameA);
      shapeA.getSize().set(1.0, 1.0, 1.0);
      shapeA.getPose().setToZero();
      SupportingFrameVertexHolder stpShapeA = new FrameSTPBox3D(shapeA);
      Node shapeNodeA = Shape3DMeshFactories.toShape3DMesh(shapeA, Color.DARKCYAN);
      Node stpShapeNodeA = Shape3DMeshFactories.toUVMesh(stpShapeA, Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2), resolution, resolution, UVMeshType.HULL);
      view3dFactory.addNodeToView(shapeNodeA);
      view3dFactory.addNodeToView(stpShapeNodeA);
      Affine nodeTransfomA = new Affine();
      shapeNodeA.getTransforms().add(nodeTransfomA);
      stpShapeNodeA.getTransforms().add(nodeTransfomA);

      FrameConvexPolytope3D shapeB = new FrameConvexPolytope3D(frameB, EuclidPolytopeFactories.newIcosahedron(1.0));
      shapeB.applyTransform(new RigidBodyTransform(new YawPitchRoll(0.35, 0.0, 0.0), new Vector3D(0.0, 0.0, 0.0)));
      SupportingFrameVertexHolder stpShapeB = new FrameSTPConvexPolytope3D(shapeB);
      Node shapeNodeB = Shape3DMeshFactories.toShape3DMesh(shapeB, Color.DARKCYAN);
      Node stpShapeNodeB = Shape3DMeshFactories.toUVMesh(stpShapeB, Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2), resolution, resolution, UVMeshType.HULL);
      view3dFactory.addNodeToView(shapeNodeB);
      view3dFactory.addNodeToView(stpShapeNodeB);
      Affine nodeTransfomB = new Affine();
      shapeNodeB.getTransforms().add(nodeTransfomB);
      stpShapeNodeB.getTransforms().add(nodeTransfomB);

      FrameGilbertJohnsonKeerthiCollisionDetector detector = new FrameGilbertJohnsonKeerthiCollisionDetector();

      new AnimationTimer()
      {
         private long lastTime = -1L;

         @Override
         public void handle(long now)
         {
            if (runButton.isSelected())
            {
               double dt = Conversions.nanosecondsToSeconds(now - lastTime);

               if (lastTime != -1L)
               {
                  if (rewindButton.isSelected())
                     dt = -dt;

                  rotationVectorA.setAndScale(dt, angularVelocityA);
                  rotationVectorB.setAndScale(dt, angularVelocityB);
                  rotationMatrixA.setRotationVector(rotationVectorA);
                  rotationMatrixB.setRotationVector(rotationVectorB);

                  transformA.getRotation().append(rotationMatrixA);
                  transformB.getRotation().append(rotationMatrixB);
                  updatePoseSpinners();
               }

               lastTime = now;
            }
            else
            {
               lastTime = -1L;
            }

            frameA.update();
            frameB.update();
            JavaFXTools.convertRigidBodyTransformToAffine(transformA, nodeTransfomA);
            JavaFXTools.convertRigidBodyTransformToAffine(transformB, nodeTransfomB);

            if (enableGJKButton.isSelected())
            {
               EuclidFrameShape3DCollisionResult result;
               detector.setInitialSupportDirection(stpInitialSupportDirection);
               result = detector.evaluateCollision(stpShapeA, stpShapeB);
               setNodeTranslate(stpPointOnANode, result.getPointOnA());
               setNodeTranslate(stpPointOnBNode, result.getPointOnB());
               stpInitialSupportDirection.setMatchingFrame(detector.getSupportDirection());
               detector.setInitialSupportDirection(initialSupportDirection);
               result = detector.evaluateCollision(shapeA, shapeB);
               setNodeTranslate(pointOnANode, result.getPointOnA());
               setNodeTranslate(pointOnBNode, result.getPointOnB());
               initialSupportDirection.setMatchingFrame(detector.getSupportDirection());
            }
         }
      }.start();

      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(new Scene(mainPane));
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
   }

   private void updatePoseSpinners()
   {
      disablePoseSpinnerListeners = true;
      positionAXSpinner.getValueFactory().setValue(transformA.getTranslationX());
      positionAYSpinner.getValueFactory().setValue(transformA.getTranslationY());
      positionAZSpinner.getValueFactory().setValue(transformA.getTranslationZ());

      positionBXSpinner.getValueFactory().setValue(transformB.getTranslationX());
      positionBYSpinner.getValueFactory().setValue(transformB.getTranslationY());
      positionBZSpinner.getValueFactory().setValue(transformB.getTranslationZ());

      yawASpinner.getValueFactory().setValue(transformA.getRotation().getYaw());
      pitchASpinner.getValueFactory().setValue(transformA.getRotation().getPitch());
      rollASpinner.getValueFactory().setValue(transformA.getRotation().getRoll());

      yawBSpinner.getValueFactory().setValue(transformB.getRotation().getYaw());
      pitchBSpinner.getValueFactory().setValue(transformB.getRotation().getPitch());
      rollBSpinner.getValueFactory().setValue(transformB.getRotation().getRoll());
      disablePoseSpinnerListeners = false;
   }

   private static void setNodeTranslate(Node node, FramePoint3DBasics point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      node.setTranslateX(point.getX());
      node.setTranslateY(point.getY());
      node.setTranslateZ(point.getZ());
   }

   @Override
   public void stop()
   {
      Platform.exit();
   }

   public static Node generateTetrahedronsMesh(List<? extends Point3DReadOnly> points, Color color, double size)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Point3DReadOnly point : points)
      {
         meshBuilder.addTetrahedron(size, point);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}