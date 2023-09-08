using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;


using System.Linq;

using KPlankton;
using KangarooSolver;
using KangarooSolver.Goals;


/// <summary>
/// This class will be instantiated on demand by the Script component.
/// </summary>
public abstract class Script_Instance_84055 : GH_ScriptInstance
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
  private void RunScript(List<Line> StartSegments, bool Reset, ref object A, ref object B)
  {

    double increment = 0.01;
    double collisionRadius = 1.0;
    double splitLength = 2.0;

    if(Reset || Springs == null)
    {
      PS = new KangarooSolver.PhysicalSystem();
      Springs = new List<IGoal>();

      foreach(Line L in StartSegments)
      {
        IGoal G = new Spring(L.From, L.To, L.Length, 1);
        PS.AssignPIndex(G, 0.0001);  //this ensures coincident endpoints are combined into single particles
        Springs.Add(G);
      }
    }

    else
    {
      int count = Springs.Count;

      for(int i = 0; i < count;i++)
      {
        Spring S = Springs[i] as Spring;
        Point3d Start = PS.GetPosition(S.PIndex[0]);
        Point3d End = PS.GetPosition(S.PIndex[1]);

        double LineLength = Start.DistanceTo(End);
        if(LineLength > splitLength)
        {
          Point3d MidPt = 0.5 * (Start + End);
          PS.AddParticle(MidPt, 1);
          int newParticleIndex = PS.ParticleCount() - 1;
          int endIndex = S.PIndex[1];
          //set the end of the original spring to be the newly created midpoint
          //and make a new spring from the midpoint to the original endpoint
          S.PIndex[1] = newParticleIndex;
          S.RestLength = 0.5 * LineLength;
          Spring otherHalf = new Spring(newParticleIndex, endIndex, 0.5 * LineLength, 1);
          Springs[i] = S;
          Springs.Add(otherHalf);
        }
        else
        {
          S.RestLength += increment;
          Springs[i] = S;
        }
      }

      //add collisions
      var GoalList = new List<IGoal>();
      GoalList.AddRange(Springs);

      var IndexList = Enumerable.Range(0, PS.ParticleCount()).ToList();
      IGoal Collisions = new SphereCollide2(IndexList, collisionRadius, 1);

      GoalList.Add(Collisions);

      PS.Step(GoalList, false, 0.001);
      A = PS.GetOutput(GoalList);
    }
  }
  #endregion
  #region Additional

  KangarooSolver.PhysicalSystem PS;
  List<IGoal> Springs;


  public class SphereCollide2 : GoalObject
  {
    public double Strength;
    public double SqDiam;
    public double Diam;

    public SphereCollide2(List<Point3d> V, double r, double k)
    {
      int L = V.Count;
      PPos = V.ToArray();
      Move = new Vector3d[L];
      Weighting = new double[L];
      for (int i = 0; i < L; i++)
      {
        Weighting[i] = k;
      }
      Diam = r + r;
      SqDiam = Diam * Diam;
      Strength = k;
    }

    public SphereCollide2(List<int> V, double r, double k)
    {
      int L = V.Count;
      PIndex = V.ToArray();
      Move = new Vector3d[L];
      Weighting = new double[L];
      for (int i = 0; i < L; i++)
      {
        Weighting[i] = k;
      }
      Diam = r + r;
      SqDiam = Diam * Diam;
      Strength = k;
    }

    public override void Calculate(List<KangarooSolver.Particle> p)
    {
      int L = PIndex.Length;
      double[] Xcoord = new double[L];
      for (int i = 0; i < L; i++)
      {
        Xcoord[i] = p[PIndex[i]].Position.X;
      }
      Array.Sort(Xcoord, PIndex);

      for (int i = 0; i < L; i++)
      {
        Move[i] = Vector3d.Zero;
        Weighting[i] = 0;
      }

      for (int i = 0; i < (PIndex.Length - 1); i++)
      {
        for (int j = 1; (i + j) < PIndex.Length; j++)
        {
          int k = i + j;
          Vector3d Separation = p[PIndex[k]].Position - p[PIndex[i]].Position;
          if (Separation.X < Diam)
          {
            if (Separation.SquareLength < SqDiam)
            {
              double LengthNow = Separation.Length;
              double stretchfactor = 1.0 - Diam / LengthNow;
              Vector3d SpringMove = 0.5 * Separation * stretchfactor;
              Move[i] += SpringMove;
              Move[k] -= SpringMove;
              Weighting[i] = Strength;
              Weighting[k] = Strength;
            }
          }

          else { break; }
        }
      }
    }

  }
  #endregion
}