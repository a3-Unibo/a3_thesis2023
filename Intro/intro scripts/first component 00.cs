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
public class Script_Instance : GH_ScriptInstance
{
#region Utility functions
  /// <summary>Print a String to the [Out] Parameter of the Script component.</summary>
  /// <param name="text">String to print.</param>
  private void Print(string text) { __out.Add(text); }
  /// <summary>Print a formatted String to the [Out] Parameter of the Script component.</summary>
  /// <param name="format">String format.</param>
  /// <param name="args">Formatting parameters.</param>
  private void Print(string format, params object[] args) { __out.Add(string.Format(format, args)); }
  /// <summary>Print useful information about an object instance to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj)); }
  /// <summary>Print the signatures of all the overloads of a specific method to the [Out] Parameter of the Script component. </summary>
  /// <param name="obj">Object instance to parse.</param>
  private void Reflect(object obj, string method_name) { __out.Add(GH_ScriptComponentUtilities.ReflectType_CS(obj, method_name)); }
#endregion

#region Members
  /// <summary>Gets the current Rhino document.</summary>
  private RhinoDoc RhinoDocument;
  /// <summary>Gets the Grasshopper document that owns this script.</summary>
  private GH_Document GrasshopperDocument;
  /// <summary>Gets the Grasshopper script component that owns this script.</summary>
  private IGH_Component Component; 
  /// <summary>
  /// Gets the current iteration count. The first call to RunScript() is associated with Iteration==0.
  /// Any subsequent call within the same solution will increment the Iteration count.
  /// </summary>
  private int Iteration;
#endregion

  /// <summary>
  /// This procedure contains the user code. Input parameters are provided as regular arguments, 
  /// Output parameters as ref arguments. You don't have to assign output parameters, 
  /// they will have a default value.
  /// </summary>
  private void RunScript(double x, double y, ref object sum, ref object diff)
  {
        // single line comment
    /*
    Multi line
    comment
    */
    // sum = x + y;
    sum = SumOfTwoNumbers(x, y);
    double s = SumOfTwoNumbers(11, 3.14);
    diff = x - y;
  }

  // <Custom additional code> 
    
  //double a;

  double SumOfTwoNumbers(double a, double b)
  {
    double c = a + b;
    return c;
  }

  double DiffOfTwoNumbers(double a, double b)
  {
    return a - b;
  }

  bool IsNumberEven(int num)
  {
    // Comparison operators
    // == equal to
    // != not equal to
    // > larger than, < smaller than
    //    if(num % 2 == 0)
    //      return true;
    //    else
    //      return false;
    return num % 2 == 0;
  }
  // </Custom additional code> 

  private List<string> __err = new List<string>(); //Do not modify this list directly.
  private List<string> __out = new List<string>(); //Do not modify this list directly.
  private RhinoDoc doc = RhinoDoc.ActiveDoc;       //Legacy field.
  private IGH_ActiveObject owner;                  //Legacy field.
  private int runCount;                            //Legacy field.
  
  public override void InvokeRunScript(IGH_Component owner, object rhinoDocument, int iteration, List<object> inputs, IGH_DataAccess DA)
  {
    //Prepare for a new run...
    //1. Reset lists
    this.__out.Clear();
    this.__err.Clear();

    this.Component = owner;
    this.Iteration = iteration;
    this.GrasshopperDocument = owner.OnPingDocument();
    this.RhinoDocument = rhinoDocument as Rhino.RhinoDoc;

    this.owner = this.Component;
    this.runCount = this.Iteration;
    this. doc = this.RhinoDocument;

    //2. Assign input parameters
        double x = default(double);
    if (inputs[0] != null)
    {
      x = (double)(inputs[0]);
    }

    double y = default(double);
    if (inputs[1] != null)
    {
      y = (double)(inputs[1]);
    }



    //3. Declare output parameters
      object sum = null;
  object diff = null;


    //4. Invoke RunScript
    RunScript(x, y, ref sum, ref diff);
      
    try
    {
      //5. Assign output parameters to component...
            if (sum != null)
      {
        if (GH_Format.TreatAsCollection(sum))
        {
          IEnumerable __enum_sum = (IEnumerable)(sum);
          DA.SetDataList(1, __enum_sum);
        }
        else
        {
          if (sum is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(1, (Grasshopper.Kernel.Data.IGH_DataTree)(sum));
          }
          else
          {
            //assign direct
            DA.SetData(1, sum);
          }
        }
      }
      else
      {
        DA.SetData(1, null);
      }
      if (diff != null)
      {
        if (GH_Format.TreatAsCollection(diff))
        {
          IEnumerable __enum_diff = (IEnumerable)(diff);
          DA.SetDataList(2, __enum_diff);
        }
        else
        {
          if (diff is Grasshopper.Kernel.Data.IGH_DataTree)
          {
            //merge tree
            DA.SetDataTree(2, (Grasshopper.Kernel.Data.IGH_DataTree)(diff));
          }
          else
          {
            //assign direct
            DA.SetData(2, diff);
          }
        }
      }
      else
      {
        DA.SetData(2, null);
      }

    }
    catch (Exception ex)
    {
      this.__err.Add(string.Format("Script exception: {0}", ex.Message));
    }
    finally
    {
      //Add errors and messages... 
      if (owner.Params.Output.Count > 0)
      {
        if (owner.Params.Output[0] is Grasshopper.Kernel.Parameters.Param_String)
        {
          List<string> __errors_plus_messages = new List<string>();
          if (this.__err != null) { __errors_plus_messages.AddRange(this.__err); }
          if (this.__out != null) { __errors_plus_messages.AddRange(this.__out); }
          if (__errors_plus_messages.Count > 0) 
            DA.SetDataList(0, __errors_plus_messages);
        }
      }
    }
  }
}