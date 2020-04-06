package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPFace3D implements Face3DReadOnly
{
   private final Face3DReadOnly owner;
   private final Vertex3DReadOnly v0, v1, v2;
   private final STPHalfEdge3D e0, e1, e2;
   private final List<STPHalfEdge3D> edges;

   public static List<STPFace3D> newSTPFace3Ds(Face3DReadOnly owner)
   {
      List<STPFace3D> faces = new ArrayList<>();

      STPFace3D previousFace = new STPFace3D(owner, owner.getVertex(0), owner.getVertex(1), owner.getVertex(2));
      faces.add(previousFace);

      for (int i = 1; i < owner.getNumberOfEdges() - 2; i++)
      {
         Vertex3DReadOnly v0 = owner.getVertex(0);
         Vertex3DReadOnly v1 = owner.getVertex(i + 1);
         Vertex3DReadOnly v2 = owner.getVertex(i + 2);
         STPFace3D currentFace = new STPFace3D(owner, v0, v1, v2);

         currentFace.e0.twin = previousFace.e2;
         previousFace.e2.twin = currentFace.e0;

         faces.add(currentFace);
      }

      return faces;
   }

   public STPFace3D(Face3DReadOnly face3D, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2)
   {
      owner = face3D;
      this.v0 = v0;
      this.v1 = v1;
      this.v2 = v2;

      e0 = new STPHalfEdge3D(v0, v1);
      e1 = new STPHalfEdge3D(v1, v2);
      e2 = new STPHalfEdge3D(v2, v0);

      e0.face = this;
      e1.face = this;
      e2.face = this;

      e0.next = e1;
      e1.next = e2;
      e2.next = e0;

      e0.next = e2;
      e1.next = e0;
      e2.next = e1;

      edges = Arrays.asList(e0, e1, e2);
   }

   public Vertex3DReadOnly getV0()
   {
      return v0;
   }
   
   public Vertex3DReadOnly getV1()
   {
      return v1;
   }

   public Vertex3DReadOnly getV2()
   {
      return v2;
   }

   public STPHalfEdge3D getE0()
   {
      return e0;
   }

   public STPHalfEdge3D getE1()
   {
      return e1;
   }

   public STPHalfEdge3D getE2()
   {
      return e2;
   }

   public boolean contains(Vertex3DReadOnly query)
   {
      return v0 == query || v1 == query || v2 == query;
   }

   @Override
   public Point3DReadOnly getCentroid()
   {
      return null;
   }

   @Override
   public Vector3DReadOnly getNormal()
   {
      return owner.getNormal();
   }

   @Override
   public double getArea()
   {
      return Double.NaN;
   }

   @Override
   public BoundingBox3DReadOnly getBoundingBox()
   {
      return null;
   }

   /**
    * Tests whether the query is located directly above or below this face, such its projection would
    * be located inside this face.
    *
    * @param query the coordinates of the query. Not modified.
    * @return {@code true} if the query is located either directly above or below this face,
    *         {@code false} otherwise.
    */
   public boolean isPointDirectlyAboveOrBelow(Point3DReadOnly query)
   {
      if (canObserverSeeEdge(query, e0))
         return false;
      if (canObserverSeeEdge(query, e1))
         return false;
      if (canObserverSeeEdge(query, e2))
         return false;
      return true;
   }

   @Override
   public List<STPHalfEdge3D> getEdges()
   {
      return edges;
   }

   public static class STPHalfEdge3D implements HalfEdge3DReadOnly
   {
      private Vertex3DReadOnly origin, destination;
      private STPFace3D face;
      private STPHalfEdge3D previous, next, twin;

      public STPHalfEdge3D(Vertex3DReadOnly origin, Vertex3DReadOnly destination)
      {
         this.origin = origin;
         this.destination = destination;
      }

      public boolean contains(Vertex3DReadOnly query)
      {
         return origin == query || destination == query;
      }

      @Override
      public Vertex3DReadOnly getOrigin()
      {
         return origin;
      }

      @Override
      public Vertex3DReadOnly getDestination()
      {
         return destination;
      }

      @Override
      public STPHalfEdge3D getTwin()
      {
         return twin;
      }

      @Override
      public STPHalfEdge3D getNext()
      {
         return next;
      }

      @Override
      public STPHalfEdge3D getPrevious()
      {
         return previous;
      }

      @Override
      public STPFace3D getFace()
      {
         return face;
      }
   }
}
