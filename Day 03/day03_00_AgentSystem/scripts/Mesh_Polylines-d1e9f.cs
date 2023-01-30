using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using System.Threading.Tasks;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_d1e9f : GH_ScriptInstance
{
  #region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { /* Implementation hidden. */ }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { /* Implementation hidden. */ }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { /* Implementation hidden. */ }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { /* Implementation hidden. */ }
  #endregion

  #region Members
  /// <summary>Gets the current Rhino document.</summary>
  private readonly RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private readonly GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private readonly IGH_Component Component;
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private readonly int Iteration;
  #endregion
  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments,
  /// Output parameters as ref arguments. You don't have to assign output parameters,
  /// they will have a default value.
  /// </summary>
  #region Runscript
  private void RunScript(List<Polyline> P, double t, ref object M)
  {
    GH_Mesh[] meshes = new GH_Mesh[P.Count];

    //for (int i=0; i< P.Count; i++) 
    Parallel.For(0, P.Count, i =>
    {
      // if your loop is parallel all extra variables used inside the loop 
      // should be locally declared
      Mesh m, mPoly = new Mesh();
      Line[] segments = P[i].GetSegments();
      Point3d a, b, c, d;

      foreach (Line line in segments)
      {
        Vector3d offsetdir = Vector3d.CrossProduct(line.Direction, Vector3d.ZAxis);
        offsetdir.Unitize();
        offsetdir *= t * 0.5;
        a = line.From + offsetdir;
        b = line.To + offsetdir;
        c = line.To - offsetdir;
        d = line.From - offsetdir;
        m = new Mesh();
        m.Vertices.AddVertices(new[] { a, b, c, d });
        m.Faces.AddFace(0, 1, 2, 3);
        m.RebuildNormals();
        mPoly.Append(m);
      }
      meshes[i] = new GH_Mesh(mPoly);
    });

    M = meshes;
  }
  #endregion
  #region Additional

  #endregion
}