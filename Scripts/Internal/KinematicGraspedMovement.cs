/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2017.                                 *
 * Leap Motion proprietary and  confidential.                                 *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Leap.Unity.Interaction {

  /// <summary>
  /// This implementation of IGraspedMovementHandler moves its interaction object to a
  /// target position and rotation by setting its rigidbody position and rotation
  /// directly to that target position and rotation. This is required when working with
  /// kinematic rigidbodies, which do not move based on their velocity and angular
  /// velocity.
  /// </summary>
  public class KinematicGraspedMovement : IGraspedMovementHandler {

    public void MoveTo(Vector3 solvedPosition, Quaternion solvedRotation,
                       InteractionBehaviour interactionObj, bool justGrasped) {
      //interactionObj.rigidbody.MovePosition(solvedPosition);
      //interactionObj.rigidbody.MoveRotation(solvedRotation);
      interactionObj.rigidbody.transform.position = solvedPosition;
      interactionObj.rigidbody.transform.rotation = solvedRotation;
      interactionObj.rigidbody.position = solvedPosition;
      interactionObj.rigidbody.rotation = solvedRotation;
    }

  }

}
