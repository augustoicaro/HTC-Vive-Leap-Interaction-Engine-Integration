/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2018.                                 *
 * Leap Motion proprietary and confidential.                                  *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using InteractionEngineUtility;
using Leap.Unity.Interaction.Internal;
using Leap.Unity.RuntimeGizmos;
using Leap.Unity.Space;
using Leap.Unity.Query;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Leap.Unity.Attributes;
using ViveHandTracking;

namespace Leap.Unity.Interaction {

    public enum HandDataSide { PlayerLeft, PlayerRight }

    [DisallowMultipleComponent]
    public class InteractionHand : InteractionController
    {
    #region Inspector

    [SerializeField]
    private GestureProvider _gestureProvider;

    [Header("Hand Configuration")]

    [Tooltip("Should the data for the underlying Leap hand come from the player's left "
           + "hand or their right hand? Alternatively, you can set this mode to Custom "
           + "to specify accessor functions manually via script (recommended for advanced "
           + "users only).")]
    [SerializeField, EditTimeOnly]
    private HandDataSide _HandDataSide;
    public HandDataSide HandDataSide {
      get { return _HandDataSide; }
      set {
        // TODO: Do validation if this is modified!
        _HandDataSide = value;
      }
    }

    /// <summary>
    /// Set slots to true to consider the corresponding finger's fingertip for primary
    /// hover checks. 0 is the thumb, 1 is the index finger, etc. Generally speaking,
    /// enable the fingertips you'd like users to be able to use to choose and push a
    /// button, but keep in mind you pay distance check costs for each fingertip enabled!
    /// </summary>
    public bool[] enabledPrimaryHoverFingertips = new bool[5] { true, true, true, false, false };

    #endregion

    #region Hand Data
    /// <summary>
    /// If the hand data mode for this InteractionHand is set to Custom, you must also
    /// manually specify the provider from which to retrieve Leap frames containing
    /// hand data.
    /// </summary>
    public GestureProvider gestureProvider {
      get { return _gestureProvider; }
      set {_gestureProvider = value; }
    }

    /// <summary>
    /// A copy of the latest tracked hand data; never null, never warped.
    /// </summary>
    private Hand _handData = new Hand();

    /// <summary>
    /// An unwarped copy of _handData (if unwarping is necessary; otherwise
    /// identical). Only relevant when using the Leap Graphical Renderer to create
    /// curved user interfaces.
    /// </summary>
    private Hand _unwarpedHandData = new Hand();

    /// <summary>
    /// Will be null when not tracked, otherwise contains the same data as _handData.
    /// </summary>
    private Hand _hand;

    #endregion

    #region Unity Events

    protected override void Reset() {
      base.Reset();

      enabledPrimaryHoverFingertips = new bool[] { true, true, true, false, false };
    }

    protected override void Start() {
      base.Start();

      // Otherwise, configure automatically.
      if (gestureProvider == null) {
        Debug.LogError("No VIVE gesture provider was found in your scene! Please "
                     + "make sure you have a LeapServiceProvider if you intend to "
                     + "use Leap hands in your scene.", this);
        return;
      }

      // Set up hover point Transform for the palm.
      Transform palmTransform = new GameObject("Palm Transform").transform;
      palmTransform.parent = this.transform;
      _backingHoverPointTransform = palmTransform;

      // Set up primary hover point Transforms for the fingertips. We'll only use
      // some of them, depending on user settings.
      for (int i = 0; i < 5; i++) {
        Transform fingertipTransform = new GameObject("Fingertip Transform").transform;
        fingertipTransform.parent = this.transform;
        _backingFingertipTransforms.Add(fingertipTransform);
        _fingertipTransforms.Add(null);
      }
    }

    void Update() {
      if (!GestureProvider.UpdatedInThisFrame)
        return;

      updateHandPosition();
    }

    private void OnDestroy() {}

    private void updateHandPosition() {
      if (!GestureProvider.UpdatedInThisFrame)
        return;

      var hand = _HandDataSide == HandDataSide.PlayerLeft ? GestureProvider.LeftHand : GestureProvider.RightHand;
      if (hand == null) {
          _hand = null;
          return;
      }

      // finger roots
      Vector3 index = hand.points[5], middle = hand.points[9], ring = hand.points[13], pinky = hand.points[17];
      Vector3 vec1 = (index + middle) / 2 - hand.points[0];
      Vector3 vec2 = (ring + pinky) / 2 - hand.points[0];
      Vector3 forward = hand.isLeft ? Vector3.Cross(vec1, vec2) : Vector3.Cross(vec2, vec1);
      Vector3 upDir = -forward;
      Vector3 rightDir = Vector3.Cross(upDir, vec1);

      // Leap hand general options
      // TODO: Review this values
      long      frameID = 1;
      int       id = hand.isLeft ? 0 : 1;
      float     confidence = 1;
      float     grabStrength = 0;
      float     grabAngle = 0;
      float     pinchStrength = 0;
      float     pinchDistance = 1;
      float     palmWidth = (index - pinky).magnitude;
      bool      isLeft = hand.isLeft;
      float     timeVisible = 1;

      // Setup fingers and bones
      float     fingerWidth = 0.02f;
      bool     isExtended = false;
      List<Finger> fingers = new List<Finger> { };
      Finger.FingerType[] fingerTypeList = {Finger.FingerType.TYPE_THUMB, Finger.FingerType.TYPE_INDEX, Finger.FingerType.TYPE_MIDDLE, Finger.FingerType.TYPE_RING, Finger.FingerType.TYPE_PINKY};
      int fingerCount = 0;

      foreach (Finger.FingerType fingerType in fingerTypeList) {
        float fingerLength = 0.0f;
        List<Bone> bones = new List<Bone> { };
        Bone.BoneType[] boneTypesList = {Bone.BoneType.TYPE_METACARPAL, Bone.BoneType.TYPE_PROXIMAL, Bone.BoneType.TYPE_INTERMEDIATE, Bone.BoneType.TYPE_DISTAL};
        float     boneWidth = 0.018f;
        int fingerRoot = 0;
        switch(fingerType) {
          case Finger.FingerType.TYPE_THUMB:
            fingerRoot = 1;
            break;

          case Finger.FingerType.TYPE_INDEX:
            fingerRoot = 5;
            break;

          case Finger.FingerType.TYPE_MIDDLE:
            fingerRoot = 9;
            break;

          case Finger.FingerType.TYPE_RING:
            fingerRoot = 13;
            break;

          case Finger.FingerType.TYPE_PINKY:
            fingerRoot = 17;
            break;
        }

        // Calculate xDir(perpendicular vector to finger's right side) for set rotation
        Vector3 secondBoneDir = hand.points[fingerRoot + 2] - hand.points[fingerRoot + 1];
        secondBoneDir = secondBoneDir.normalized;
        Vector3 xDir = Vector3.Cross(secondBoneDir, rightDir);
        xDir = Vector3.Cross(secondBoneDir, xDir);
        Vector3 xDirR = xDir;
        if (fingerType == Finger.FingerType.TYPE_THUMB)
            xDirR = Quaternion.AngleAxis(-90, secondBoneDir) * xDir;


        int boneIndex = 0;
        foreach (Bone.BoneType boneType in boneTypesList) {
          Vector prevJoint;
          Vector nextJoint;
          Vector3 boneDir;
          if (boneType == Bone.BoneType.TYPE_METACARPAL) {
            Vector3 handDownDirection = hand.position - middle;
            handDownDirection = handDownDirection.normalized;
            Vector3 rootThumb = hand.points[1] + handDownDirection*0.025f;
            Vector3 jointRoot = Vector3.Lerp(rootThumb, hand.points[0], (float)(fingerRoot - 1) / 16);
            jointRoot = Vector3.Lerp(jointRoot, hand.points[fingerRoot + 1], 0.2f);
            prevJoint = new Vector(jointRoot.x, jointRoot.y, jointRoot.z);
            nextJoint = new Vector(hand.points[fingerRoot + boneIndex].x, hand.points[fingerRoot + boneIndex].y, hand.points[fingerRoot + boneIndex].z);
            boneDir = hand.points[fingerRoot + boneIndex] - jointRoot;
            boneDir = boneDir.normalized;
            if (fingerType == Finger.FingerType.TYPE_THUMB)
              prevJoint = nextJoint;
          }
          else {
            prevJoint = new Vector(hand.points[fingerRoot + boneIndex].x, hand.points[fingerRoot + boneIndex].y, hand.points[fingerRoot + boneIndex].z);
            boneIndex++;
            nextJoint = new Vector(hand.points[fingerRoot + boneIndex].x, hand.points[fingerRoot + boneIndex].y, hand.points[fingerRoot + boneIndex].z);
            boneDir = hand.points[fingerRoot + boneIndex] - hand.points[fingerRoot + boneIndex - 1];
            boneDir = boneDir.normalized;
          }

          float boneLength = (nextJoint-prevJoint).Magnitude;
          fingerLength += boneLength;
          Vector center = (nextJoint + prevJoint)/ 2 ;
          Vector boneDirection = nextJoint - prevJoint;
          boneDirection = boneDirection.Normalized;

          Vector3 boneUpDir = Vector3.Cross(xDirR, boneDir);
          if (fingerType == Finger.FingerType.TYPE_THUMB && boneType == Bone.BoneType.TYPE_METACARPAL)
              boneUpDir = Vector3.Cross((Quaternion.AngleAxis(70, secondBoneDir) * xDir), boneDir);
          Quaternion boneRotationUnity = Quaternion.LookRotation(boneDir, boneUpDir);
          LeapQuaternion boneRotation = new LeapQuaternion(boneRotationUnity.x, boneRotationUnity.y, boneRotationUnity.z, boneRotationUnity.w);
          bones.Add(new Bone( prevJoint,
                              nextJoint,
                              center,
                              boneDirection,
                              boneLength,
                              boneWidth,
                              boneType,
                              boneRotation));
        }

        Vector fingerTipPosition = bones[3].NextJoint;
        Vector fingerDirection = bones[3].NextJoint - bones[3].PrevJoint;
        fingerDirection = fingerDirection.Normalized;
        fingers.Add( new Finger( 0,
                                 id,
                                 fingerCount,
                                 timeVisible,
                                 fingerTipPosition,
                                 fingerDirection,
                                 fingerWidth,
                                 fingerLength,
                                 isExtended,
                                 fingerType,
                                 bones[0],
                                 bones[1],
                                 bones[2],
                                 bones[3]));
        fingerCount++;
      }


      Vector     palmPosition = new Vector(hand.position.x, hand.position.y, hand.position.z);
      Vector     stabilizedPalmPosition = new Vector(hand.position.x, hand.position.y, hand.position.z);
      Vector     palmVelocity = new Vector(0,0,0);
      Vector     palmNormal = new Vector(forward.x, forward.y, forward.z);
      palmNormal = palmNormal.Normalized;

      Vector3 midDir = (middle+ring)/2 - hand.points[0];
      Quaternion m_NewRot = Quaternion.LookRotation(midDir, upDir);
      LeapQuaternion palmOrientation = new LeapQuaternion(m_NewRot.x, m_NewRot.y, m_NewRot.z, m_NewRot.w);

      Vector direction = new Vector((middle - hand.position).x, (middle - hand.position).y, (middle - hand.position).z);
      direction = direction.Normalized;
      Vector     wristPosition = new Vector(hand.points[0].x, hand.points[0].y, hand.points[0].z);

      Vector elbow = wristPosition - direction*0.1f;
      Vector wrist = wristPosition;
      Vector armCenter = (elbow+wristPosition)/2;
      float length = 0.1f;
      float width = palmWidth;
      Vector armDirection = direction;
      LeapQuaternion armRotation = palmOrientation;
      Arm arm = new Arm(elbow, wrist, armCenter, armDirection, length, width, armRotation);

      _hand = new Hand( frameID,
                        id,
                        confidence,
                        grabStrength,
                        grabAngle,
                        pinchStrength,
                        pinchDistance,
                        palmWidth,
                        isLeft,
                        timeVisible,
                        arm,
                        fingers,
                        palmPosition,
                        stabilizedPalmPosition,
                        palmVelocity,
                        palmNormal,
                        palmOrientation,
                        direction,
                        wristPosition );

      _handData.CopyFrom(_hand);
      _unwarpedHandData.CopyFrom(_handData);

      refreshPointDataFromHand();
      _lastCustomHandWasLeft = _unwarpedHandData.IsLeft;

    }

    #endregion

    #region General InteractionController Implementation

    /// <summary>
    /// Gets whether the underlying Leap hand is currently tracked.
    /// </summary>
    public override bool isTracked { get { return _hand != null; } }

    /// <summary>
    /// Gets whether the underlying Leap hand is currently being moved in worldspace.
    /// </summary>
    public override bool isBeingMoved { get { return isTracked; } }

    /// <summary>
    /// Gets the last tracked state of the Leap hand.
    ///
    /// Note for those using the Leap Graphical Renderer: If the hand required warping
    /// due to the nearby presence of an object in warped (curved) space, this will
    /// return the hand as warped from that object's curved space into the rectilinear
    /// space containing its colliders. This is only relevant if you are using the Leap
    /// Graphical Renderer to render curved, interactive objects.
    /// </summary>
    public Hand leapHand { get { return _unwarpedHandData; } }

    private bool _lastCustomHandWasLeft = false;
    /// <summary>
    /// Gets whether the underlying tracked Leap hand is a left hand.
    /// </summary>
    public override bool isLeft {
      get {
        switch (HandDataSide) {
          case HandDataSide.PlayerLeft:
            return true;
          case HandDataSide.PlayerRight:
            return false;
          default:
            return true;
        }
      }
    }

    /// <summary>
    /// Gets the last-tracked position of the underlying Leap hand.
    /// </summary>
    public override Vector3 position {
      get { return _handData.PalmPosition.ToVector3(); }
    }

    /// <summary>
    /// Gets the last-tracked rotation of the underlying Leap hand.
    /// </summary>
    public override Quaternion rotation {
      get { return _handData.Rotation.ToQuaternion(); }
    }

    /// <summary>
    /// Gets the velocity of the underlying tracked Leap hand.
    /// </summary>
    public override Vector3 velocity {
      get { return isTracked ? leapHand.PalmVelocity.ToVector3() : Vector3.zero; }
    }

    /// <summary>
    /// Gets the controller type of this InteractionControllerBase. InteractionHands
    /// are Interaction Engine controllers implemented over Leap hands.
    /// </summary>
    public override ControllerType controllerType {
      get { return ControllerType.Hand; }
    }

    /// <summary>
    /// Returns this InteractionHand object. This property will be null if the
    /// InteractionControllerBase is not ControllerType.Hand.
    /// </summary>
    public override InteractionHand intHand {
      get { return this; }
    }

    protected override void onObjectUnregistered(IInteractionBehaviour intObj) {
      grabClassifier.UnregisterInteractionBehaviour(intObj);
    }

    protected override void fixedUpdateController() {
      // Transform the hand ahead if the manager is in a moving reference frame.
      if (manager.hasMovingFrameOfReference) {
        Vector3 transformAheadPosition;
        Quaternion transformAheadRotation;
        manager.TransformAheadByFixedUpdate(_unwarpedHandData.PalmPosition.ToVector3(),
                                            _unwarpedHandData.Rotation.ToQuaternion(),
                                            out transformAheadPosition,
                                            out transformAheadRotation);
        _unwarpedHandData.SetTransform(transformAheadPosition, transformAheadRotation);
      }
    }

    #endregion

    #region Hovering Controller Implementation

    private Transform _backingHoverPointTransform = null;
    public override Vector3 hoverPoint {
      get {
        if (_backingHoverPointTransform == null) {
          return leapHand.PalmPosition.ToVector3();
        }
        else {
          return _backingHoverPointTransform.position;
        }
      }
    }

    private List<Transform> _backingFingertipTransforms = new List<Transform>();
    private List<Transform> _fingertipTransforms = new List<Transform>();
    protected override List<Transform> _primaryHoverPoints {
      get {
        return _fingertipTransforms;
      }
    }

    private void refreshPointDataFromHand() {
      refreshHoverPoint();
      refreshPrimaryHoverPoints();
      refreshGraspManipulatorPoints();
    }

    private void refreshHoverPoint() {
      _backingHoverPointTransform.position = leapHand.PalmPosition.ToVector3();
      _backingHoverPointTransform.rotation = leapHand.Rotation.ToQuaternion();
    }

    private void refreshPrimaryHoverPoints() {
      for (int i = 0; i < enabledPrimaryHoverFingertips.Length; i++) {
        if (enabledPrimaryHoverFingertips[i]) {
          _fingertipTransforms[i] = _backingFingertipTransforms[i];

          Finger finger = leapHand.Fingers[i];
          _fingertipTransforms[i].position = finger.TipPosition.ToVector3();
          _fingertipTransforms[i].rotation = finger.bones[3].Rotation.ToQuaternion();
        }
        else {
          _fingertipTransforms[i] = null;
        }
      }
    }

    protected override void unwarpColliders(Transform primaryHoverPoint,
                                            ISpaceComponent warpedSpaceElement) {
      // Extension method calculates "unwarped" pose in world space.
      Vector3    unwarpedPosition;
      Quaternion unwarpedRotation;
      warpedSpaceElement.anchor.transformer.WorldSpaceUnwarp(primaryHoverPoint.position,
                                                             primaryHoverPoint.rotation,
                                                             out unwarpedPosition,
                                                             out unwarpedRotation);

      // First shift the hand to be centered on the fingertip position so that rotations
      // applied to the hand will pivot around the fingertip, then apply the rest of the
      // transformation.
      _unwarpedHandData.Transform(-primaryHoverPoint.position, Quaternion.identity);
      _unwarpedHandData.Transform(unwarpedPosition, unwarpedRotation
                                                    * Quaternion.Inverse(primaryHoverPoint.rotation));

      // Hand data was modified, so refresh point data.
      refreshPointDataFromHand();
    }

    #endregion

    #region Contact Controller Implementation

    private const int NUM_FINGERS = 5;
    private const int BONES_PER_FINGER = 3;

    private ContactBone[] _contactBones;
    public override ContactBone[] contactBones {
      get { return _contactBones; }
    }

    private GameObject _contactBoneParent;
    protected override GameObject contactBoneParent {
      get { return _contactBoneParent; }
    }

    private delegate void BoneMapFunc(Leap.Hand hand, out Vector3 targetPosition,
                                                      out Quaternion targetRotation);
    private BoneMapFunc[] _handContactBoneMapFunctions;

    protected override void getColliderBoneTargetPositionRotation(int contactBoneIndex,
                                                                  out Vector3 targetPosition,
                                                                  out Quaternion targetRotation) {
      using (new ProfilerSample("InteractionHand: getColliderBoneTargetPositionRotation")) {
        _handContactBoneMapFunctions[contactBoneIndex](_unwarpedHandData,
                                                       out targetPosition,
                                                       out targetRotation);
      }
    }

    protected override bool initContact() {
      if (!isTracked) return false;

      initContactBoneContainer();
      initContactBones();

      return true;
    }

    protected override void onPreEnableSoftContact() {
      resetContactBoneJoints();
    }

    protected override void onPostDisableSoftContact() {
      if (isTracked) resetContactBoneJoints();
    }

    #region Contact Bone Management

    private void initContactBoneContainer() {
      string name = (_unwarpedHandData.IsLeft ? "Left" : "Right") + " Interaction Hand Contact Bones";
      _contactBoneParent = new GameObject(name);
    }

    private void initContactBones() {
      _contactBones = new ContactBone[NUM_FINGERS * BONES_PER_FINGER + 1];
      _handContactBoneMapFunctions = new BoneMapFunc[NUM_FINGERS * BONES_PER_FINGER + 1];

      // Finger bones
      for (int fingerIndex = 0; fingerIndex < NUM_FINGERS; fingerIndex++) {
        for (int jointIndex = 0; jointIndex < BONES_PER_FINGER; jointIndex++) {
          GameObject contactBoneObj = new GameObject("Contact Fingerbone", typeof(CapsuleCollider), typeof(Rigidbody), typeof(ContactBone));
          contactBoneObj.layer = manager.contactBoneLayer;

          Bone bone = _unwarpedHandData.Fingers[fingerIndex]
                                       .Bone((Bone.BoneType)(jointIndex) + 1); // +1 to skip first bone.
          int boneArrayIndex = fingerIndex * BONES_PER_FINGER + jointIndex;
          contactBoneObj.transform.position = bone.Center.ToVector3();
          contactBoneObj.transform.rotation = bone.Rotation.ToQuaternion();

          // Remember the method we used to calculate this bone position from
          // a Leap Hand for later.
          int fingerIndexCopy = fingerIndex;
          int jointIndexCopy = jointIndex;
          _handContactBoneMapFunctions[boneArrayIndex] = (Leap.Hand hand,
                                                          out Vector3 targetPosition,
                                                          out Quaternion targetRotation) => {
            Bone theBone = hand.Fingers[fingerIndexCopy].Bone((Bone.BoneType)(jointIndexCopy + 1));
            targetPosition = theBone.Center.ToVector3();
            targetRotation = theBone.Rotation.ToQuaternion();
          };

          CapsuleCollider capsule = contactBoneObj.GetComponent<CapsuleCollider>();
          capsule.direction = 2;
          capsule.radius = bone.Width * 0.5f;
          capsule.height = bone.Length + bone.Width;
          capsule.material = defaultContactBoneMaterial;

          ContactBone contactBone = initContactBone(bone, contactBoneObj, boneArrayIndex, capsule);

          contactBone.lastTargetPosition = bone.Center.ToVector3();
        }
      }

      // Palm bone
      {
        // Palm is attached to the third metacarpal and derived from it.
        GameObject contactBoneObj = new GameObject("Contact Palm Bone", typeof(BoxCollider), typeof(Rigidbody), typeof(ContactBone));

        Bone bone = _unwarpedHandData.Fingers[(int)Finger.FingerType.TYPE_MIDDLE].Bone(Bone.BoneType.TYPE_METACARPAL);
        int boneArrayIndex = NUM_FINGERS * BONES_PER_FINGER;
        contactBoneObj.transform.position = _unwarpedHandData.PalmPosition.ToVector3();
        contactBoneObj.transform.rotation = _unwarpedHandData.Rotation.ToQuaternion();

        // Remember the method we used to calculate the palm from a Leap Hand for later.
        _handContactBoneMapFunctions[boneArrayIndex] = (Leap.Hand hand,
                                                        out Vector3 targetPosition,
                                                        out Quaternion targetRotation) => {
          targetPosition = hand.PalmPosition.ToVector3();
          targetRotation = hand.Rotation.ToQuaternion();
        };

        BoxCollider box = contactBoneObj.GetComponent<BoxCollider>();
        box.center = new Vector3(_unwarpedHandData.IsLeft ? -0.005f : 0.005f, bone.Width * -0.3f, -0.01f);
        box.size = new Vector3(bone.Length, bone.Width, bone.Length);
        box.material = defaultContactBoneMaterial;

        initContactBone(null, contactBoneObj, boneArrayIndex, box);
      }

      // Constrain the bones to each other to prevent separation.
      addContactBoneJoints();
    }

    private ContactBone initContactBone(Leap.Bone bone, GameObject contactBoneObj, int boneArrayIndex, Collider boneCollider) {
      contactBoneObj.layer = _contactBoneParent.gameObject.layer;
      contactBoneObj.transform.localScale = Vector3.one;

      ContactBone contactBone = contactBoneObj.GetComponent<ContactBone>();
      contactBone.collider = boneCollider;
      contactBone.interactionController = this;
      contactBone.interactionHand = this;
      _contactBones[boneArrayIndex] = contactBone;

      Transform capsuleTransform = contactBoneObj.transform;
      capsuleTransform.SetParent(_contactBoneParent.transform, false);

      Rigidbody body = contactBoneObj.GetComponent<Rigidbody>();
      body.freezeRotation = true;
      contactBone.rigidbody = body;
      body.useGravity = false;
      body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic; // TODO: Allow different collision detection modes as an optimization.

      body.mass = 0.1f;
      body.position = bone != null ? bone.Center.ToVector3()
                                   : _unwarpedHandData.PalmPosition.ToVector3();
      body.rotation = bone != null ? bone.Rotation.ToQuaternion()
                                   : _unwarpedHandData.Rotation.ToQuaternion();
      contactBone.lastTargetPosition = bone != null ? bone.Center.ToVector3()
                                            : _unwarpedHandData.PalmPosition.ToVector3();

      return contactBone;
    }

    private void addContactBoneJoints() {
      for (int fingerIndex = 0; fingerIndex < NUM_FINGERS; fingerIndex++) {
        for (int jointIndex = 0; jointIndex < BONES_PER_FINGER; jointIndex++) {
          Bone bone = _unwarpedHandData.Fingers[fingerIndex].Bone((Bone.BoneType)(jointIndex) + 1); // +1 to skip first bone.
          int boneArrayIndex = fingerIndex * BONES_PER_FINGER + jointIndex;

          FixedJoint joint = _contactBones[boneArrayIndex].gameObject.AddComponent<FixedJoint>();
          joint.autoConfigureConnectedAnchor = false;
          if (jointIndex != 0) {
            Bone prevBone = _unwarpedHandData.Fingers[fingerIndex].Bone((Bone.BoneType)(jointIndex));
            joint.connectedBody = _contactBones[boneArrayIndex - 1].rigidbody;
            joint.anchor = Vector3.back * bone.Length / 2f;
            joint.connectedAnchor = Vector3.forward * prevBone.Length / 2f;
            _contactBones[boneArrayIndex].joint = joint;
          }
          else {
            joint.connectedBody = _contactBones[NUM_FINGERS * BONES_PER_FINGER].rigidbody;
            joint.anchor = Vector3.back * bone.Length / 2f;
            joint.connectedAnchor = _contactBones[NUM_FINGERS * BONES_PER_FINGER].transform.InverseTransformPoint(bone.PrevJoint.ToVector3());
            _contactBones[boneArrayIndex].metacarpalJoint = joint;
          }
        }
      }
    }

    /// <summary> Reconnects and resets all the joints in the hand. </summary>
    private void resetContactBoneJoints() {
      // If contact bones array is null, there's nothing to reset. This can happen if
      // the controller is disabled before it had a chance to initialize contact.
      if (_contactBones == null) return;

      // If the palm contact bone is null, we can't reset bone joints.
      if (_contactBones[NUM_FINGERS * BONES_PER_FINGER] == null) return;

      _contactBones[NUM_FINGERS * BONES_PER_FINGER].transform.position = _unwarpedHandData.PalmPosition.ToVector3();
      _contactBones[NUM_FINGERS * BONES_PER_FINGER].transform.rotation = _unwarpedHandData.Rotation.ToQuaternion();
      _contactBones[NUM_FINGERS * BONES_PER_FINGER].rigidbody.velocity = Vector3.zero;
      _contactBones[NUM_FINGERS * BONES_PER_FINGER].rigidbody.angularVelocity = Vector3.zero;

      for (int fingerIndex = 0; fingerIndex < NUM_FINGERS; fingerIndex++) {
        for (int jointIndex = 0; jointIndex < BONES_PER_FINGER; jointIndex++) {
          Bone bone = _unwarpedHandData.Fingers[fingerIndex].Bone((Bone.BoneType)(jointIndex) + 1); // +1 to skip first bone.
          int boneArrayIndex = fingerIndex * BONES_PER_FINGER + jointIndex;

          _contactBones[boneArrayIndex].transform.position = bone.Center.ToVector3();
          _contactBones[boneArrayIndex].transform.rotation = bone.Rotation.ToQuaternion();
          _contactBones[boneArrayIndex].rigidbody.position = bone.Center.ToVector3();
          _contactBones[boneArrayIndex].rigidbody.rotation = bone.Rotation.ToQuaternion();
          _contactBones[boneArrayIndex].rigidbody.velocity = Vector3.zero;
          _contactBones[boneArrayIndex].rigidbody.angularVelocity = Vector3.zero;

          if (jointIndex != 0 && _contactBones[boneArrayIndex].joint != null) {
            Bone prevBone = _unwarpedHandData.Fingers[fingerIndex].Bone((Bone.BoneType)(jointIndex));
            _contactBones[boneArrayIndex].joint.connectedBody = _contactBones[boneArrayIndex - 1].rigidbody;
            _contactBones[boneArrayIndex].joint.anchor = Vector3.back * bone.Length / 2f;
            _contactBones[boneArrayIndex].joint.connectedAnchor = Vector3.forward * prevBone.Length / 2f;
          }
          else if (_contactBones[boneArrayIndex].metacarpalJoint != null) {
            _contactBones[boneArrayIndex].metacarpalJoint.connectedBody = _contactBones[NUM_FINGERS * BONES_PER_FINGER].rigidbody;
            _contactBones[boneArrayIndex].metacarpalJoint.anchor = Vector3.back * bone.Length / 2f;
            _contactBones[boneArrayIndex].metacarpalJoint.connectedAnchor = _contactBones[NUM_FINGERS * BONES_PER_FINGER].transform
                                                                            .InverseTransformPoint(bone.PrevJoint.ToVector3());
          }
        }
      }
    }

    /// <summary>
    /// A utility function that sets a Hand object's bones based on this InteractionHand.
    /// Can be used to display a graphical hand that matches the physical one.
    /// </summary>
    public void FillBones(Hand inHand) {
      if (softContactEnabled) { return; }
      if (Application.isPlaying && _contactBones.Length == NUM_FINGERS * BONES_PER_FINGER + 1) {
        Vector elbowPos = inHand.Arm.ElbowPosition;
        inHand.SetTransform(_contactBones[NUM_FINGERS * BONES_PER_FINGER].rigidbody.position, _contactBones[NUM_FINGERS * BONES_PER_FINGER].rigidbody.rotation);

        for (int fingerIndex = 0; fingerIndex < NUM_FINGERS; fingerIndex++) {
          for (int jointIndex = 0; jointIndex < BONES_PER_FINGER; jointIndex++) {
            Bone bone = inHand.Fingers[fingerIndex].Bone((Bone.BoneType)(jointIndex) + 1);
            int boneArrayIndex = fingerIndex * BONES_PER_FINGER + jointIndex;
            Vector displacement = _contactBones[boneArrayIndex].rigidbody.position.ToVector() - bone.Center;
            bone.Center += displacement;
            bone.PrevJoint += displacement;
            bone.NextJoint += displacement;
            bone.Rotation = _contactBones[boneArrayIndex].rigidbody.rotation.ToLeapQuaternion();
          }
        }

        inHand.Arm.PrevJoint = elbowPos;
        inHand.Arm.Direction = (inHand.Arm.PrevJoint - inHand.Arm.NextJoint).Normalized;
        inHand.Arm.Center = (inHand.Arm.PrevJoint + inHand.Arm.NextJoint) * 0.5f;
      }
    }

    #endregion

    #endregion

    #region Grasp Controller Implementation

    private List<Vector3> _graspManipulatorPoints = new List<Vector3>();
    public override List<Vector3> graspManipulatorPoints {
      get {
        return _graspManipulatorPoints;
      }
    }

    private void refreshGraspManipulatorPoints() {
      int bufferIndex = 0;
      for (int i = 0; i < NUM_FINGERS; i++) {
        for (int boneIdx = 0; boneIdx < 2; boneIdx++) {
          // Update or add knuckle-joint and first-finger-bone positions as the grasp
          // manipulator points for this Hand.

          Vector3 point = leapHand.Fingers[i].bones[boneIdx].NextJoint.ToVector3();

          if (_graspManipulatorPoints.Count - 1 < bufferIndex) {
            _graspManipulatorPoints.Add(point);
          }
          else {
            _graspManipulatorPoints[bufferIndex] = point;
          }
          bufferIndex += 1;
        }
      }
    }

    private HeuristicGrabClassifier _grabClassifier;
    /// <summary>
    /// Handles logic determining whether a hand has grabbed or released an interaction object.
    /// </summary>
    public HeuristicGrabClassifier grabClassifier {
      get {
        if (_grabClassifier == null) _grabClassifier = new HeuristicGrabClassifier(this);
        return _grabClassifier;
      }
    }

    private Vector3[] _fingertipPositionsBuffer = new Vector3[5];
    /// <summary>
    /// Returns approximately where the controller is grasping the currently-grasped
    /// InteractionBehaviour. Specifically, returns the average position of all grasping
    /// fingertips of the InteractionHand.
    ///
    /// This method will print an error if the hand is not currently grasping an object.
    /// </summary>
    public override Vector3 GetGraspPoint() {
      if (!isGraspingObject) {
        Debug.LogError("Tried to get grasp point of InteractionHand, but it is not "
                     + "currently grasping an object.", this);
        return leapHand.PalmPosition.ToVector3();
      }

      int numGraspingFingertips = 0;
      _grabClassifier.GetGraspingFingertipPositions(graspedObject, _fingertipPositionsBuffer, out numGraspingFingertips);
      if (numGraspingFingertips > 0) {
        Vector3 sum = Vector3.zero;
        for (int i = 0; i < numGraspingFingertips; i++) {
          sum += _fingertipPositionsBuffer[i];
        }
        return sum / numGraspingFingertips;
      }
      else {
        return leapHand.PalmPosition.ToVector3();
      }
    }

    /// <summary>
    /// Attempts to manually initiate a grasp on the argument interaction object. A grasp
    /// will only begin if a finger and thumb are both in contact with the interaction
    /// object. If this method successfully initiates a grasp, it will return true,
    /// otherwise it will return false.
    /// </summary>
    protected override bool checkShouldGraspAtemporal(IInteractionBehaviour intObj) {
      if (grabClassifier.TryGrasp(intObj, leapHand)) {
        var tempControllers = Pool<List<InteractionController>>.Spawn();
        try {
          tempControllers.Add(this);
          intObj.BeginGrasp(tempControllers);
          return true;
        }
        finally {
          tempControllers.Clear();
          Pool<List<InteractionController>>.Recycle(tempControllers);
        }
      }

      return false;
    }

    public override void SwapGrasp(IInteractionBehaviour replacement) {
      var original = graspedObject;

      base.SwapGrasp(replacement);

      grabClassifier.SwapClassifierState(original, replacement);
    }

    protected override void fixedUpdateGraspingState() {
      grabClassifier.FixedUpdateClassifierHandState();
    }

    protected override void onGraspedObjectForciblyReleased(IInteractionBehaviour objectToBeReleased) {
      grabClassifier.NotifyGraspForciblyReleased(objectToBeReleased);
    }

    protected override bool checkShouldRelease(out IInteractionBehaviour objectToRelease) {
      return grabClassifier.FixedUpdateClassifierRelease(out objectToRelease);
    }

    protected override bool checkShouldGrasp(out IInteractionBehaviour objectToGrasp) {
      return grabClassifier.FixedUpdateClassifierGrasp(out objectToGrasp);
    }

    #endregion

    #region Gizmos

    #if UNITY_EDITOR

    private Leap.Hand _testHand = null;

    public override void OnDrawRuntimeGizmos(RuntimeGizmoDrawer drawer) {
      if (Application.isPlaying) {
        base.OnDrawRuntimeGizmos(drawer);
      }
      else {
        var provider = gestureProvider;

        if (_testHand == null && provider != null) {
          _testHand = _hand;
        }

        // Hover Point
        _unwarpedHandData = _testHand; // hoverPoint is driven by this backing variable
        drawHoverPoint(drawer, hoverPoint);

        // Primary Hover Points
        for (int i = 0; i < NUM_FINGERS; i++) {
          if (enabledPrimaryHoverFingertips[i]) {
            drawPrimaryHoverPoint(drawer, _testHand.Fingers[i].TipPosition.ToVector3());
          }
        }
      }
    }

    #endif

    #endregion

    }
}
