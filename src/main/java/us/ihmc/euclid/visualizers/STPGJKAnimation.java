package us.ihmc.euclid.visualizers;

import java.util.List;
import java.util.Random;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
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
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
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
   private ToggleButton runButton, rewindButton;
   @FXML
   private Spinner<Double> angularVelocityAX, angularVelocityAY, angularVelocityAZ;
   @FXML
   private Spinner<Double> angularVelocityBX, angularVelocityBY, angularVelocityBZ;

   private final Vector3D angularVelocityA = new Vector3D(0.1, 0.2, 0.15);
   private final Vector3D angularVelocityB = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D rotationVectorA = new Vector3D(1.0, 0.0, 0.0);
   private final Vector3D rotationVectorB = new Vector3D(0.0, 0.0, 0.0);
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

   public void initialize()
   {
      angularVelocityAX.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityA.getX(), 0.05));
      angularVelocityAY.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityA.getY(), 0.05));
      angularVelocityAZ.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityA.getZ(), 0.05));

      angularVelocityBX.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityB.getX(), 0.05));
      angularVelocityBY.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityB.getY(), 0.05));
      angularVelocityBZ.setValueFactory(new DoubleSpinnerValueFactory(-5.0, 5.0, angularVelocityB.getZ(), 0.05));

      angularVelocityAX.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityA.setX(newValue));
      angularVelocityAY.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityA.setY(newValue));
      angularVelocityAZ.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityA.setZ(newValue));

      angularVelocityBX.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityB.setX(newValue));
      angularVelocityBY.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityB.setY(newValue));
      angularVelocityBZ.getValueFactory().valueProperty().addListener((o, oldValue, newValue) -> angularVelocityB.setZ(newValue));

      frameA.update();
      frameB.update();
   }

   @FXML
   public void reset()
   {
      transformA.getRotation().setToZero();
      transformB.getRotation().setToZero();
      frameA.update();
      frameB.update();
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
            if (!runButton.isSelected())
            {
               JavaFXTools.convertRigidBodyTransformToAffine(transformA, nodeTransfomA);
               JavaFXTools.convertRigidBodyTransformToAffine(transformB, nodeTransfomB);

               lastTime = -1L;
               return;
            }
            double dt = Conversions.nanosecondsToSeconds(now - lastTime);
            lastTime = now;

            if (lastTime == -1L)
               return;

            if (rewindButton.isSelected())
               dt = -dt;

            rotationVectorA.setAndScale(dt, angularVelocityA);
            rotationVectorB.setAndScale(dt, angularVelocityB);
            rotationMatrixA.setRotationVector(rotationVectorA);
            rotationMatrixB.setRotationVector(rotationVectorB);

            transformA.getRotation().append(rotationMatrixA);
            transformB.getRotation().append(rotationMatrixB);

            frameA.update();
            frameB.update();

            JavaFXTools.convertRigidBodyTransformToAffine(transformA, nodeTransfomA);
            JavaFXTools.convertRigidBodyTransformToAffine(transformB, nodeTransfomB);

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
      }.start();

      mainPane.setCenter(view3dFactory.getSubSceneWrappedInsidePane());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(new Scene(mainPane));
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
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