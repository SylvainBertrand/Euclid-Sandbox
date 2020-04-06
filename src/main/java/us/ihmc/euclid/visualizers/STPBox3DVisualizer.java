package us.ihmc.euclid.visualizers;

import java.util.List;
import java.util.Random;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.visualizers.Shape3DMeshFactories.UVMeshType;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class STPBox3DVisualizer extends Application
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

      Random random = new Random(3406);
      Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
      box3D.getPose().setToZero();
      STPBox3D stpBox = new STPBox3D(box3D);
      view3dFactory.addNodeToView(Shape3DMeshFactories.toBox3DMesh(box3D, Color.DARKCYAN));
      int resolution = 150;
      view3dFactory.addNodeToView(Shape3DMeshFactories.toUVMesh(stpBox,
                                                                Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2),
                                                                resolution,
                                                                resolution,
                                                                UVMeshType.HULL));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toUVMesh(stpBox,
                                                                Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2),
                                                                resolution,
                                                                resolution,
                                                                UVMeshType.SUPPORT_DIRECTIONS));

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