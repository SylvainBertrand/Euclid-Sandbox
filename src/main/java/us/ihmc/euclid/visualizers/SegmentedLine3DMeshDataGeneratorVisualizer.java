package us.ihmc.euclid.visualizers;

import java.util.Arrays;
import java.util.Random;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class SegmentedLine3DMeshDataGeneratorVisualizer extends Application
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

      int numberOfWaypoints = 15;
      Random random = new Random(546651);

      for (int i = 0; i < 50; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.nextLineSegment3D(random);
         lineSegment3D.translate(EuclidCoreRandomTools.nextPoint3D(random, 3.0));

         Point3DBasics[] waypoints = new Point3DBasics[numberOfWaypoints];
         for (int waypointIndex = 0; waypointIndex < numberOfWaypoints; waypointIndex++)
         {
            double alpha = waypointIndex / (numberOfWaypoints - 1.0);
            waypoints[waypointIndex] = lineSegment3D.pointOnLineGivenPercentage(alpha);
         }
         SegmentedLine3DMeshDataGenerator generator = new SegmentedLine3DMeshDataGenerator(numberOfWaypoints, 8, 0.01);
         generator.compute(waypoints);
         JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
         Arrays.asList(generator.getMeshDataHolders()).forEach(mesh -> meshBuilder.addMesh(mesh));
         MeshView meshView = new MeshView(meshBuilder.generateMesh());
         meshView.setMaterial(new PhongMaterial(Color.BURLYWOOD));
         view3dFactory.addNodeToView(meshView);
      }

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
