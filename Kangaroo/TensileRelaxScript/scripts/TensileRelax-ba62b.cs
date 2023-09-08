using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;


using KPlankton;
using KangarooSolver;
using KangarooSolver.Goals;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_ba62b : GH_ScriptInstance
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
  private void RunScript(Mesh M, ref object A, ref object B)
  {

    //initialize the solver
    var PS = new KangarooSolver.PhysicalSystem();
    var Goals = new List<IGoal>();

    //get the Mesh points and boundary status
    Point3d[] Pts = M.Vertices.ToPoint3dArray();
    bool[] Naked = M.GetNakedEdgePointStatus();

    for(int i = 0; i < M.Vertices.Count;i++)
    {
      PS.AddParticle(Pts[i], 1); //add a particle for every mesh vertex
      if(Naked[i])
      {Goals.Add(new KangarooSolver.Goals.Anchor(i, Pts[i], 10000));}// fix the boundaries strongly in place
    }

    for(int i = 0; i < M.TopologyEdges.Count;i++)
    {
      var Ends = M.TopologyEdges.GetTopologyVertices(i);
      int Start = M.TopologyVertices.MeshVertexIndices(Ends.I)[0];
      int End = M.TopologyVertices.MeshVertexIndices(Ends.J)[0];
      Goals.Add(new KangarooSolver.Goals.Spring(Start, End, 0, 1)); //for each edge, a spring with rest length 0, and strength 1
    }

    int counter = 0;
    double threshold = 1e-6;
    do
    {
      //Step forward, using these goals, with multi-threading on, and stopping if the threshold is reached
      PS.Step(Goals, true, threshold);
      counter++;
    }while(PS.GetvSum() > threshold && counter < 100);
    //GetvSum returns the current kinetic energy
    //always include a counter to prevent it getting stuck forever in case it cannot reach the given threshold

    //replace the mesh vertices with the relaxed ones
    M.Vertices.Clear();
    M.Vertices.AddVertices(PS.GetPositions());

    //Output the mesh, and how many iterations it took to converge
    A = M;
    B = counter;
  }
  #endregion
  #region Additional

  #endregion
}