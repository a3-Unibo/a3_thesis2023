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
public abstract class Script_Instance_48aba : GH_ScriptInstance
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
  private void RunScript(bool reset, bool go, int nV, Point3d P, Vector3d V, double rad, ref object Pos, ref object Vel, ref object r)
  {
    // ---------------------------------------- 00 reset & initialize
    if (reset || myVehicleSimulation == null)
    {
      myVehicleSimulation = new VehicleSimulation(nV, P, V, rad);
    }

    // ---------------------------------------- 01 simulate
    if (go)
    {
      myVehicleSimulation.Update();
      Component.ExpireSolution(true);
    }

    // ---------------------------------------- output
    List<Point3d> out_pos = new List<Point3d>();
    List<Vector3d> out_vel = new List<Vector3d>();
    List<double> out_rad = new List<double>();

    foreach (Vehicle v in myVehicleSimulation.vehicles)
    {
      out_pos.Add(v.position);
      out_vel.Add(v.velocity);
      out_rad.Add(v.radius);
    }

    //_V = myVehicles;
    Pos = out_pos;
    Vel = out_vel;
    r = out_rad;
  }
  #endregion
  #region Additional

  // global variables
  VehicleSimulation myVehicleSimulation;
  //List<Vehicle> myVehicles;

  // simulation classes
  public class VehicleSimulation
  {
    public List<Vehicle> vehicles;

    public VehicleSimulation(int nVehicles, Point3d P, Vector3d V, double rad)
    {
      // initialize list
      vehicles = new List<Vehicle>();
      // declare random variable
      Random rnd = new Random();
      for (int i = 0; i < nVehicles; i++)
      {
        Vector3d vel = V;
        vel.Rotate(rnd.NextDouble() * Math.PI * 2, new Vector3d(0, 0, 1));
        double radius = rad * rnd.NextDouble() + 0.1;
        Vehicle v = new Vehicle(P, vel, radius, i); // instantiation
        vehicles.Add(v);
      }
    }

    public void Update()
    {
      foreach (Vehicle v in vehicles) v.Move();
    }
  }

  public class Vehicle
  {
    // fields
    public Point3d position;
    public Vector3d velocity;
    public double radius;
    Random rnd;

    // constructors
    public Vehicle()
    {
      position = Point3d.Origin;  // new Point3d();
      velocity = Vector3d.XAxis;
      radius = 1;
    }

    public Vehicle(Point3d p, Vector3d v, double r, int seed)
    {
      position = p;
      velocity = v;
      radius = r;
      rnd = new Random(seed);
    }

    // methods
    public void Move()
    {
      //p = p + v; // this doesn't work!!!
      //position = position + velocity;
      //                    random -1 <-> 1      *     maxAngle
      double angle = ((rnd.NextDouble() - 0.5) * 2) * (0.25 * Math.PI);
      velocity.Rotate(angle, Vector3d.ZAxis);
      position += velocity; // -=  *= /=
    }
  }
  #endregion
}