/*!
 * Rapid trajectory generation for quadrocopters
 *
 * Copyright 2014 by Mark W. Mueller <mwm@mwm.im>
 *
 * This code is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This code is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the code.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RapidTrajectoryGenerator2D.h"
#include "RootFinder/quartic.hpp"

#include <algorithm>
#include <limits>

using namespace RapidQuadrocopterTrajectoryGenerator;

RapidTrajectoryGenerator2D::RapidTrajectoryGenerator2D(const Vec2 x0, const Vec2 v0, const Vec2 a0, const Vec2 gravity)
{
  //initialise each axis:
  Reset();
  for(int i=0; i<2; i++) _axis[i].SetInitialState(x0[i],v0[i],a0[i]);
  _grav = gravity;
}

void RapidTrajectoryGenerator2D::SetGoalPosition(const Vec2 in)
{
  for(unsigned i=0;i<2;i++) SetGoalPositionInAxis(i, in[i]);
}

void RapidTrajectoryGenerator2D::SetGoalVelocity(const Vec2 in)
{
  for(int i=0;i<2;i++) SetGoalVelocityInAxis(i, in[i]);
}

void RapidTrajectoryGenerator2D::SetGoalAcceleration(const Vec2 in)
{
  for(int i=0;i<2;i++) SetGoalAccelerationInAxis(i, in[i]);
}

void RapidTrajectoryGenerator2D::Reset(void)
{
  for(int i=0; i<2; i++)
  {
    _axis[i].Reset();
  }
  _tf = 0;
}

//Generate the trajectory:
void RapidTrajectoryGenerator2D::Generate(const double timeToFinish)
{
  _tf = timeToFinish;
  for(int i=0;i<2;i++)
  {
    _axis[i].GenerateTrajectory(_tf);
  }
}

RapidTrajectoryGenerator2D::InputFeasibilityResult RapidTrajectoryGenerator2D::CheckInputFeasibilitySection(double fminAllowed, double fmaxAllowed, double wmaxAllowed, double t1, double t2, double minTimeSection)
{
  if (t2 - t1 < minTimeSection) return InputIndeterminable;
  //test the acceleration at the two limits:
  if (std::max(GetThrust(t1), GetThrust(t2)) > fmaxAllowed) return InputInfeasibleThrustHigh;
  if (std::min(GetThrust(t1), GetThrust(t2)) < fminAllowed) return InputInfeasibleThrustLow;

  double fminSqr = 0;
  double fmaxSqr = 0;
  double jmaxSqr = 0;

  //Test the limits of the box we're putting around the trajectory:
  for (int i = 0; i < 2; i++)
  {
    double amin, amax;
    _axis[i].GetMinMaxAcc(amin, amax, t1, t2);

    //distance from zero thrust point in this axis
    double v1 = amin - _grav[i]; //left
    double v2 = amax - _grav[i]; //right

    //definitely infeasible:
    if (std::max(pow(v1, 2), pow(v2, 2)) > pow(fmaxAllowed, 2)) return InputInfeasibleThrustHigh;

    if (v1 * v2 < 0)
    {
      //sign of acceleration changes, so we've gone through zero
      fminSqr += 0;
    }
    else
    {
      fminSqr += pow(std::min(fabs(v1), fabs(v2)), 2);
    }

    fmaxSqr += pow(std::max(fabs(v1), fabs(v2)), 2);

    jmaxSqr += _axis[i].GetMaxJerkSquared(t1, t2);
  }

  double fmin = sqrt(fminSqr);
  double fmax = sqrt(fmaxSqr);
  double wBound;
  if (fminSqr > 1e-6) wBound = sqrt(jmaxSqr / fminSqr);//the 1e-6 is a divide-by-zero protection
  else wBound = std::numeric_limits<double>::max();

  //definitely infeasible:
  if (fmax < fminAllowed) return InputInfeasibleThrustLow;
  if (fmin > fmaxAllowed) return InputInfeasibleThrustHigh;

  //possibly infeasible:
  if (fmin < fminAllowed || fmax > fmaxAllowed || wBound > wmaxAllowed)
  { //indeterminate: must check more closely:
    double tHalf = (t1 + t2) / 2;
    InputFeasibilityResult r1 = CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1, tHalf, minTimeSection);

		if(r1 == InputFeasible)
		{
      //continue with second half
      return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, tHalf, t2, minTimeSection);
		}

		//first section is already infeasible, or indeterminate:
		return r1;
  }

  //definitely feasible:
  return InputFeasible;
}

RapidTrajectoryGenerator2D::InputFeasibilityResult RapidTrajectoryGenerator2D::CheckInputFeasibility(double fminAllowed, double fmaxAllowed, double wmaxAllowed, double minTimeSection)
{
  //required thrust limits along trajectory
  double t1 = 0;
  double t2 = _tf;

  return CheckInputFeasibilitySection(fminAllowed, fmaxAllowed, wmaxAllowed, t1, t2, minTimeSection);
}
