/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2017.                                 *
 * Leap Motion proprietary and  confidential.                                 *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using InteractionEngineUtility;
using Leap.Unity.Query;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Leap.Unity.Interaction {

  /// <summary>
  /// This implementation of IGraspedMovementHandler moves an interaction object to its
  /// target position and rotation by setting its rigidbody's velocity and angular
  /// velocity such that it will reach the target position and rotation on the next
  /// physics update.
  /// </summary>
  public class NonKinematicGraspedMovement : IGraspedMovementHandler {

    protected float _maxVelocity = 6F;

    private bool _useLastSolvedCoMPosition = false;
    private Vector3 _lastSolvedCoMPosition = Vector3.zero;
    private Vector3 _lastSolvedPosition = Vector3.zero;
    protected AnimationCurve _strengthByDistance = new AnimationCurve(new Keyframe(0.0f, 1.0f, 0.0f, 0.0f),
                                                                      new Keyframe(0.02f, 0.3f, 0.0f, 0.0f));

    public void MoveTo(Vector3 solvedPosition, Quaternion solvedRotation,
                       InteractionBehaviour intObj, bool justGrasped) {
      Vector3 solvedCenterOfMass = solvedRotation * intObj.rigidbody.centerOfMass + solvedPosition;
      Vector3 currCenterOfMass = intObj.rigidbody.rotation * intObj.rigidbody.centerOfMass + intObj.rigidbody.position;

      Vector3 targetVelocity = PhysicsUtility.ToLinearVelocity(currCenterOfMass, solvedCenterOfMass, Time.fixedDeltaTime);
      Vector3 targetAngularVelocity = PhysicsUtility.ToAngularVelocity(intObj.rigidbody.rotation, solvedRotation, Time.fixedDeltaTime);

      // Clamp targetVelocity by _maxVelocity on **CoMTargetVelocity**.
      //float maxScaledVelocity = _maxVelocity * intObj.manager.SimulationScale;
      //float targetSpeedSqrd = CoMTargetVelocity.sqrMagnitude;
      //if (targetSpeedSqrd > maxScaledVelocity * maxScaledVelocity) {
      //  float targetPercent = maxScaledVelocity / Mathf.Sqrt(targetSpeedSqrd);
      //  targetVelocity *= targetPercent;
      //  targetAngularVelocity *= targetPercent;
      //}

      _useLastSolvedCoMPosition = !justGrasped;
      float followStrength = 1F;
      if (_useLastSolvedCoMPosition) {
        float remainingDistanceLastFrame = Vector3.Distance(_lastSolvedCoMPosition, currCenterOfMass);
        followStrength = _strengthByDistance.Evaluate(remainingDistanceLastFrame / intObj.manager.SimulationScale);
      }

      Vector3 lerpedVelocity = Vector3.Lerp(intObj.rigidbody.velocity, targetVelocity, followStrength);
      Vector3 lerpedAngularVelocity = Vector3.Lerp(intObj.rigidbody.angularVelocity, targetAngularVelocity, followStrength);

      intObj.rigidbody.velocity = lerpedVelocity;
      intObj.rigidbody.angularVelocity = lerpedAngularVelocity;

      _lastSolvedPosition = solvedPosition;
      _lastSolvedCoMPosition = currCenterOfMass;
    }
  }

}
