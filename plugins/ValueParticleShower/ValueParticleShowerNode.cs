using System;
using System.ComponentModel.Composition;
using SlimDX;
using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;
using VVVV.Utils.SlimDX;

using VVVV.Core.Logging;

namespace VVVV.Nodes
{
	[PluginInfo(Name = "ParticleShower", Category = "Value")]
	public class ValueParticleShowerNode : IPluginEvaluate
	{
		[Input("Base Point")]
		ISpread<Vector3D> FBasePoint;
		
		[Input("Base Direction")]
		ISpread<Vector3D> FBaseDirection;
		
		[Input("Energy", DefaultValue = 1.0)]
		ISpread<double> FEnergyIn;
		
		[Input("Radio Length", DefaultValue = 1.0)]
		ISpread<double> FRadioLengthIn;

		[Input("Calculate Next Branch", IsBang = true)]
		ISpread<bool> FCalculateNextBranch;

		[Input("Reset", IsBang = true)]
		ISpread<bool> FResetIn;
		
		[Input("X Degree")]
		ISpread<int> FXDegreeIn;
		
		[Input("Z Degree")]
		ISpread<int> FZDegreeIn;
		
		[Input("Min Energy", DefaultValue = 0.01)]
		ISpread<double> FMinEnergyIn;
		
		[Output("Element Path Start")]
		ISpread<ISpread<Vector3D>> FElementPathStartOut;

		[Output("Element Path End")]
		ISpread<ISpread<Vector3D>>FElementPathEndOut;
		
		[Output("Energy")]
		ISpread<ISpread<double>> FEnergyOut;
		
		[Output("Type")]
		ISpread<ISpread<string>> FTypeOut;

		[Import()]
		ILogger FLogger;
		
		private Spread<Line> FBranchLines = new Spread<Line>();
		private Random FRandomGenerator = new Random();

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			if(FResetIn[0]) Clean();

			if(FCalculateNextBranch[0]) CalculateNextBranch();
		}

		private void Clean()
		{
			FElementPathStartOut.SliceCount = 0;
			FElementPathEndOut.SliceCount = 0;
			FEnergyOut.SliceCount = 0;
			FTypeOut.SliceCount = 0;
			
			FBranchLines.SliceCount = 0;
			
			var spread = new Spread<Vector3D>(1);
			spread[0] = FBasePoint[0];
			FElementPathStartOut.Add(spread);
			
			var spreadEnd = new Spread<Vector3D>(1);
			spreadEnd[0] = FBasePoint[0] + FBaseDirection[0] * 0.5;
			FElementPathEndOut.Add(spreadEnd);
			
			var energySpread = new Spread<double>(1);
			energySpread[0] = FEnergyIn[0];
			FEnergyOut.Add(energySpread);
			
			var typeSpread = new Spread<string>(1);
			typeSpread[0] = LineType.Electron.ToString();
			
			FTypeOut.Add(typeSpread);
			
			FBranchLines.Add(new Line(FElementPathStartOut[0][0], FElementPathEndOut[0][0], FEnergyIn[0], LineType.Electron));
		}

		private void CalculateNextBranch()
		{
			var branchLines = FBranchLines.Clone();
			FBranchLines.SliceCount = 0;
			
			foreach(var startLine in branchLines)
			{
				var startPoint = startLine.End;
				var direction = (startLine.End - startLine.Start);
				var startAxis = direction;
				direction = ~direction;
				
				var baseEnergy = startLine.Energy;
				
				var typeA = LineType.Electron;
				var typeB = LineType.Electron;
				
				var energyB = 0.0;
				
				switch(startLine.Type)
				{
					case LineType.Electron:
					case LineType.Positron:
						typeB = LineType.Photon;
						energyB = baseEnergy * 0.2;
					break;
					case LineType.Photon:
						typeB = LineType.Positron;
						energyB = baseEnergy * 0.5;
					break;
				}
				
				var energyA = baseEnergy - energyB;
				
				var endA = startPoint + direction * (FEnergyIn[0] - energyA);
				var endB = startPoint + direction * (FEnergyIn[0] - energyB);
				
				var vectorA = endA - startPoint;
				var vectorB = endB - startPoint;
				
				var perpA = new Vector3D(1, 1, -(vectorA.x + vectorA.y) / vectorA.z);
				var perpB = new Vector3D(1, 1, -(vectorB.x + vectorB.y) / vectorB.z);
				
				var rotQuaternionA = Quaternion.RotationAxis(perpA.ToSlimDXVector(), (float) (FZDegreeIn[0] * (1 - energyA) * FRandomGenerator.NextDouble() * VMath.DegToRad));
				var rotQuaternionB = Quaternion.RotationAxis(perpB.ToSlimDXVector(), (float) -(FZDegreeIn[0] * (1 - energyB) * FRandomGenerator.NextDouble() * VMath.DegToRad));
				
				endA = Matrix.RotationQuaternion(rotQuaternionA).ToMatrix4x4() * (endA - startPoint) + startPoint;
				endB = Matrix.RotationQuaternion(rotQuaternionB).ToMatrix4x4() * (endB - startPoint) + startPoint;

				var wholeRotationQuat = Quaternion.RotationAxis(startAxis.ToSlimDXVector(), (float) (FRandomGenerator.Next(-FXDegreeIn[0]/2, FXDegreeIn[0]/2) * VMath.DegToRad));
				var wholeRotation = Matrix.RotationQuaternion(wholeRotationQuat).ToMatrix4x4();
				
				endA = wholeRotation * (endA - startPoint) + startPoint;
				endB = wholeRotation * (endB - startPoint) + startPoint;

				if(energyA >= FMinEnergyIn[0]) 
				{
					FBranchLines.Add(new Line(startPoint, endA, energyA, typeA));
				}
				
				if(energyB >= FMinEnergyIn[0])
				{
					FBranchLines.Add(new Line(startPoint, endB, energyB, typeB));
				}
				
				Vector3D vecA = endA - startPoint;
				var lengthA = vecA.Length;
				
				Vector3D vecB = endB - startPoint;
				var lengthB = vecB.Length;
				
				int[] indexes = new int[2]{0, 1};
				
				if(lengthB > lengthA) indexes = new int[2]{1, 0};
				
				var startPointSpread = new Spread<Vector3D>();
				startPointSpread.Add(startPoint);
				startPointSpread.Add(startPoint);
				
				var endPointSpread = new Spread<Vector3D>(2);
				endPointSpread[indexes[0]] = endA;
				endPointSpread[indexes[1]] = endB;
				
				var energySpread = new Spread<double>(2);
				energySpread[indexes[0]] = energyA;
				energySpread[indexes[1]] = energyB;
				
				var typeSpread = new Spread<string>(2);
				typeSpread[indexes[0]] = typeA.ToString();
				typeSpread[indexes[1]] = typeB.ToString();
				
				FElementPathStartOut.Add(startPointSpread);
				FElementPathEndOut.Add(endPointSpread);
				FEnergyOut.Add(energySpread);
				FTypeOut.Add(typeSpread);
			}
		}
	}
	
	public struct Line
	{
		public Line(Vector3D start, Vector3D end, double energy, LineType type)
		{
			Start = start; 
			End = end;
			Energy = energy;
			Type = type;	
		}
		
		public Vector3D Start;
		public Vector3D End;
		public Double Energy;
		public LineType Type;
	}
	
	public enum LineType
	{
		Electron,
		Positron,
		Photon
	}
}