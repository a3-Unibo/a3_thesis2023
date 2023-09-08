using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;



/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_2de64 : GH_ScriptInstance
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
  private void RunScript(bool reset, bool go, List<Point3d> P, int maxCount, ref object Pc)
  {
    // initialization
    if (reset || centers == null)
      centers = new List<Point3d>(P);

    List<Vector3d> totalMoves = new List<Vector3d>();
    List<double> collisionCounts = new List<double>();

    for (int i = 0; i < centers.Count; i++)
    {
      totalMoves.Add(new Vector3d(0.0, 0.0, 0.0));
      collisionCounts.Add(0.0);
    }

    // Update
    double collisionDistance = 2.0;
    double growthDistance = 1.99;

    if (go)
    {

      // compute collisions
      for (int i = 0; i < centers.Count; i++)
        for (int j = i + 1; j < centers.Count; j++)
        {
          double d = centers[i].DistanceTo(centers[j]);
          if (d > collisionDistance) continue;
          Vector3d move = centers[i] - centers[j];
          if (move.Length < 0.001) continue;
          move.Unitize();
          move *= 0.5 * (collisionDistance - d);
          totalMoves[i] += move;
          totalMoves[j] -= move;
          collisionCounts[i] += 1.0;
          collisionCounts[j] += 1.0;
        }
      // update positions
      for (int i = 0; i < centers.Count; i++)
        if (collisionCounts[i] != 0.0)
          centers[i] += totalMoves[i] / collisionCounts[i];
      // growth
      if (centers.Count < maxCount)
      {
        for(int i=0; i< centers.Count -1; i++)
          if (centers[i].DistanceTo(centers[i + 1]) > growthDistance)
          {
            Point3d newCenter = (centers[i] + centers[i + 1]) * 0.5;
            centers.Insert(i+1, newCenter);
          }
      }


      // refresh component
      Component.ExpireSolution(true);
    }

    // Output
    Pc = centers;
  }
  #endregion
  #region Additional
  List<Point3d> centers;
  #endregion
}