/******************************************************************************
 * Copyright (C) Leap Motion, Inc. 2011-2017.                                 *
 * Leap Motion proprietary and  confidential.                                 *
 *                                                                            *
 * Use subject to the terms of the Leap Motion SDK Agreement available at     *
 * https://developer.leapmotion.com/sdk_agreement, or another agreement       *
 * between Leap Motion and you, your company or other organization.           *
 ******************************************************************************/

using InteractionEngineUtility;
using Leap.Unity.Attributes;
using Leap.Unity.RuntimeGizmos;
using Leap.Unity.Query;
using Leap.Unity.Interaction.Internal;
using System;
using System.Collections;
using System.Collections.Generic;
#if UNITY_EDITOR
using UnityEditor;
#endif
using UnityEngine;

namespace Leap.Unity.Interaction {

  [Serializable]
  public class InteractionControllerSet : SerializableHashSet<InteractionController> { }

  [DisallowMultipleComponent]
  [ExecuteInEditMode]
  public class InteractionManager : MonoBehaviour, IRuntimeGizmoComponent {

    // Header "Interaction Controllers" via InteractionManagerEditor.cs.
    [SerializeField]
    private InteractionControllerSet _interactionControllers = new InteractionControllerSet();
    /// <summary>
    /// Gets the list of interaction controllers managed by this InteractionManager.
    /// </summary>
    public ReadonlyHashSet<InteractionController> interactionControllers {
      get { return _interactionControllers; }
    }

    [Header("Interaction Settings")]

    [SerializeField]
    [Tooltip("Beyond this radius, an interaction object will not receive hover or primary "
           + "hover callbacks from an interaction controller. (Smaller values are "
           + "cheaper.) This value is automatically scaled under the hood by the "
           + "Interaction Manager's lossyScale.x, so it's recommended to keep your "
           + "Interaction Manager with unit scale underneath your 'Player' Transform if "
           + "you expect your player's hands or controllers to ever have non-unit scale.")]
    public float hoverActivationRadius = 0.2F;

    [Tooltip("Beyond this radius, an interaction object will not be considered for "
           + "contact or grasping logic. The radius should be small as an optimization "
           + "but certainly not smaller than an interaction controller and not too tight "
           + "around the controller to allow good behavior when it is moving quickly "
           + "through space. This value is automatically scaled under the hood by the "
           + "Interaction Manager's lossyScale.x, so it's recommended to keep your "
           + "Interaction Manager with unit scale underneath your 'Player' Transform if "
           + "you expect your player's hands or controllers to ever have non-unit scale.")]
    public float touchActivationRadius = 0.075F;

    [Header("Layer Settings")]
    [Tooltip("Whether or not to create the layers used for interaction when the scene "
           + "runs. Interactions require an interaction layer (for objects), a grasped "
           + "object layer, and a contact bone layer (for interaction controller 'bone'"
           + "colliders). Keep this checked to have these layers created for you, but be "
           + "aware that the generated layers will have blank names due to Unity "
           + "limitations.")]
    [SerializeField]
    [EditTimeOnly]
    protected bool _autoGenerateLayers = true;
    /// <summary>
    /// Gets whether auto-generate layers was enabled for this Interaction Manager.
    /// </summary>
    public bool autoGenerateLayers { get { return _autoGenerateLayers; } }

    [Tooltip("When automatically generating layers, the Interaction layer (for "
           + "interactable objects) will use the same physics collision flags as the "
           + "layer specified here.")]
    [SerializeField]
    protected SingleLayer _templateLayer = 0;
    public SingleLayer templateLayer { get { return _templateLayer; } }

    [Tooltip("The layer for interactable objects (i.e. InteractionBehaviours). Usually "
           + "this would have the same collision flags as the Default layer, but it "
           + "should be its own layer so interaction controllers don't have to check "
           + "collision against all physics objects in the scene.")]
    [SerializeField]
    protected SingleLayer _interactionLayer = 0;
    public SingleLayer interactionLayer { get { return _interactionLayer; } }

    [Tooltip("The layer objects are moved to when they become grasped, or if they are "
           + "otherwise ignoring controller contact. This layer should not collide with "
           + "the contact bone layer, but should collide with everything else that the "
           + "interaction layer collides with.")]
    [SerializeField]
    protected SingleLayer _interactionNoContactLayer = 0;
    public SingleLayer interactionNoContactLayer { get { return _interactionNoContactLayer; } }

    [Tooltip("The layer containing the collider 'bones' of the interaction controller. "
           + "This layer should collide with anything you'd like to be able to touch, "
           + "but it should not collide with the grasped object layer.")]
    [SerializeField]
    protected SingleLayer _contactBoneLayer = 0;
    public SingleLayer contactBoneLayer { get { return _contactBoneLayer; } }

    [Header("Debug Settings")]
    [SerializeField]
    [Tooltip("Rendering runtime gizmos requires having a Runtime Gizmo Manager somewhere "
           + "in the scene.")]
    private bool _drawControllerRuntimeGizmos = false;

    public Action OnGraphicalUpdate = () => { };
    public Action OnPrePhysicalUpdate = () => { };
    public Action OnPostPhysicalUpdate = () => { };

    private float _scale = 1F;

    /// <summary>
    /// Interaction objects further than this distance from a given controller's hover
    /// point will not be considered for any hover interactions with that controller.
    /// </summary>
    public float WorldHoverActivationRadius { get { return hoverActivationRadius * _scale; } }

    /// <summary>
    /// Interaction objects further than this distance from a given controller's hover
    /// point will not be considered for any contact or grasping interactions with that
    /// controller.
    /// </summary>
    public float WorldTouchActivationRadius { get { return touchActivationRadius * _scale; } }

    /// <summary>
    /// A scale that can be used to appropriately transform distances that otherwise expect
    /// one Unity unit to correspond to one meter.
    /// </summary>
    public float SimulationScale { get { return _scale; } }

    private HashSet<IInteractionBehaviour> _interactionObjects = new HashSet<IInteractionBehaviour>();
    /// <summary>
    /// Gets a set of all interaction objects currently registered with this Interaction 
    /// Manager.
    /// </summary>
    public ReadonlyHashSet<IInteractionBehaviour> interactionObjects {
      get { return _interactionObjects; }
    }

    private Dictionary<Rigidbody, IInteractionBehaviour> _interactionObjectBodies;
    /// <summary>
    /// Maps a Rigidbody to its attached interaction object, if the Rigidbody is part of
    /// and interaction object.
    /// </summary>
    public Dictionary<Rigidbody, IInteractionBehaviour> interactionObjectBodies {
      get {
        if (_interactionObjectBodies == null) {
          _interactionObjectBodies = new Dictionary<Rigidbody, IInteractionBehaviour>();
        }
        return _interactionObjectBodies;
      }
    }

    private Dictionary<Rigidbody, ContactBone> _contactBoneBodies;
    /// <summary>
    /// Maps a Rigidbody to its attached ContactBone, if the Rigidbody is part of an 
    /// interaction controller.
    /// </summary>
    public Dictionary<Rigidbody, ContactBone> contactBoneBodies {
      get {
        if (_contactBoneBodies == null) {
          _contactBoneBodies = new Dictionary<Rigidbody, ContactBone>();
        }
        return _contactBoneBodies;
      }
    }

    private static InteractionManager s_instance;
    /// <summary> Often, only one InteractionManager is necessary per Unity scene.
    /// This property will contain that InteractionManager as soon as its Awake()
    /// method is called. Using more than one InteractionManager is valid, but be
    /// sure to assign any InteractionBehaviour's desired manager appropriately.
    /// </summary>
    /// 
    /// <remarks> By default, this static property contains the first InteractionManager
    /// that has had its Awake() method called in the current scene. If an
    /// InteractionBehaviourBase does not have a non-null interactionManager by the
    /// time it has Start() called, it will default to using the InteractionManager
    /// referenced here.
    /// 
    /// If you have multiple InteractionManagers in your scene, you should be sure to
    /// assign InteractionBehaviours' managers appropriately. If you instantiate an
    /// InteractionBehaviour at runtime, you should assign its InteractionManager
    /// right after you instantiate it. </remarks>
    public static InteractionManager instance {
      get {
        if (s_instance == null) { s_instance = FindObjectOfType<InteractionManager>(); }
        return s_instance;
      }
      set { s_instance = value; }
    }

    void OnValidate() {
      if (!Application.isPlaying && _autoGenerateLayers) {
        generateAutomaticLayers();
      }

      refreshInteractionControllers();
    }

    void Awake() {
#if UNITY_EDITOR
      if (InteractionPreferences.shouldPrompForGravity && Application.isPlaying) {
        float magnitude = Physics.gravity.y;
        if (Mathf.Abs(magnitude) > InteractionPreferences.MAX_GRAVITY_MAGNITUDE) {
          if (!EditorUtility.DisplayDialog("Gravity magnitude too strong!", "Your gravity magnitude is " + magnitude + " which is stronger than the recommended value of -4.905!\n\nGo to Edit->Project Settings->Physics to change the magnitude.", "Ok", "Don't Show Again")) {
            InteractionPreferences.shouldPrompForGravity = false;
          }
          EditorApplication.isPlaying = false;
          return;
        }
      }

      if (InteractionPreferences.shouldPrompForPhysicsTimestep && Application.isPlaying) {
        if (Time.fixedDeltaTime > InteractionPreferences.MAX_TIMESTEP + Mathf.Epsilon) {
          float roundedTimestep = (float)Math.Round(InteractionPreferences.MAX_TIMESTEP, 4);
          if (!EditorUtility.DisplayDialog("Timestep too slow!", "Your fixed timestep is " + Time.fixedDeltaTime + ", which is slower than the recommended value of " + roundedTimestep + ".\n\nGo to Edit->ProjectSettings->Time to change the fixed timestep.", "Ok", "Don't Show Again")) {
            InteractionPreferences.shouldPrompForPhysicsTimestep = false;
          }
          EditorApplication.isPlaying = false;
          return;
        }
      }
#endif

      refreshInteractionControllers();

      if (!Application.isPlaying) return;

      if (s_instance == null) s_instance = this;

      if (_autoGenerateLayers) {
        generateAutomaticLayers();
        setupAutomaticCollisionLayers();
      }

      _prevPosition = this.transform.position;
      _prevRotation = this.transform.rotation;

#if UNITY_EDITOR
      if (_drawControllerRuntimeGizmos == true) {
        if (FindObjectOfType<RuntimeGizmoManager>() == null) {
          Debug.LogWarning("'_drawControllerRuntimeGizmos' is enabled, but there is no "
                         + "RuntimeGizmoManager in your scene. Please add one if you'd "
                         + "like to render gizmos in the editor and in your headset.");
        }
      }
#endif
    }

    void OnDisable() {
#if UNITY_EDITOR
      if (!Application.isPlaying) return;
#endif

      foreach (var intController in _interactionControllers) {
        // Disables the colliders in the interaction controller;
        // soft contact won't be applied if the controller is not updating.
        intController.EnableSoftContact();

        if (intController.isGraspingObject) {
          intController.ReleaseGrasp();
        }
      }
    }

    void Update() {
#if UNITY_EDITOR
      refreshInteractionControllers();
#endif
    }

    void FixedUpdate() {
      OnPrePhysicalUpdate();

      refreshInteractionControllers();

#if UNITY_EDITOR
      if (!Application.isPlaying) return;
#endif

      using (new ProfilerSample("Interaction Manager FixedUpdate", this.gameObject)) {
        // Ensure scale information is up-to-date.
        _scale = this.transform.lossyScale.x;

        // Update each interaction controller (Leap hands or supported VR controllers).
        fixedUpdateInteractionControllers();

        // Perform each interaction object's FixedUpdateObject.
        using (new ProfilerSample("FixedUpdateObject per-InteractionBehaviour")) {
          foreach (var interactionObj in _interactionObjects) {
            interactionObj.FixedUpdateObject();
          }
        }

        // Apply soft contacts from all controllers in a unified solve.
        // (This will clear softContacts and originalVelocities as well.)
        using (new ProfilerSample("Apply Soft Contacts")) {
          if (_softContacts.Count > 0) {
            PhysicsUtility.applySoftContacts(_softContacts, _softContactOriginalVelocities);
          }
        }
      }

      OnPostPhysicalUpdate();

      updateMovingFrameOfReferenceSupport();
    }

    void LateUpdate() {
      OnGraphicalUpdate();
    }

    #region Controller Interaction State & Callbacks Update

    private HashSet<InteractionController> _activeControllersBuffer = new HashSet<InteractionController>();
    private HashSet<InteractionController> _hoverControllersBuffer = new HashSet<InteractionController>();
    private HashSet<InteractionController> _contactControllersBuffer = new HashSet<InteractionController>();
    private HashSet<InteractionController> _graspingControllersBuffer = new HashSet<InteractionController>();

    private void fixedUpdateInteractionControllers() {

      _hoverControllersBuffer.Clear();
      _contactControllersBuffer.Clear();
      _graspingControllersBuffer.Clear();
      _activeControllersBuffer.Clear();
      foreach (var controller in interactionControllers) {
        if (!controller.isActiveAndEnabled) continue;

        _activeControllersBuffer.Add(controller);

        if (controller.hoverEnabled) _hoverControllersBuffer.Add(controller);
        if (controller.contactEnabled) _contactControllersBuffer.Add(controller);
        if (controller.graspingEnabled) _graspingControllersBuffer.Add(controller);
      }

      using (new ProfilerSample("Fixed Update Controllers (General Update)")) {
        // Perform general controller update, for controller collider and point
        // representations.
        foreach (var controller in _activeControllersBuffer) {
          if (!controller.isActiveAndEnabled) continue;
          (controller as IInternalInteractionController).FixedUpdateController();
        }
      }

      using (new ProfilerSample("Fixed Update Controllers (Interaction State and Callbacks)")) {

        /* 
         * Interactions are checked here in a very specific manner so that interaction
         * callbacks always occur in a strict order and interaction object state is
         * always updated directly before the relevant callbacks occur.
         * 
         * Interaction callbacks will only occur outside this order if a script
         * manually forces interaction state-changes; for example, calling
         * interactionController.ReleaseGrasp() will immediately call
         * interactionObject.OnPerControllerGraspEnd() on the formerly grasped object.
         * 
         * Callback order:
         * - Suspension (when a grasped object's grasping controller loses tracking)
         * - Just-Ended Interactions (Grasps, then Contacts, then Hovers)
         * - Just-Begun Interactions (Hovers, then Contacts, then Grasps)
         * - Sustained Interactions (Hovers, then Contacts, then Grasps)
         */

        // Suspension //

        // Check controllers beginning object suspension.
        foreach (var controller in _graspingControllersBuffer) {
          IInteractionBehaviour suspendedObj;
          if ((controller as IInternalInteractionController).CheckSuspensionBegin(out suspendedObj)) {
            suspendedObj.BeginSuspension(controller);
          }
        }

        // Check controllers ending object suspension.
        foreach (var controller in _graspingControllersBuffer) {
          IInteractionBehaviour resumedObj;
          if ((controller as IInternalInteractionController).CheckSuspensionEnd(out resumedObj)) {
            resumedObj.EndSuspension(controller);
          }
        }

        // Ending Interactions //

        checkEndingGrasps(_graspingControllersBuffer);
        checkEndingContacts(_contactControllersBuffer);
        checkEndingPrimaryHovers(_hoverControllersBuffer);
        checkEndingHovers(_hoverControllersBuffer);

        // Beginning Interactions //

        checkBeginningHovers(_hoverControllersBuffer);
        checkBeginningPrimaryHovers(_hoverControllersBuffer);
        checkBeginningContacts(_contactControllersBuffer);
        checkBeginningGrasps(_graspingControllersBuffer);

        // Sustained Interactions //

        checkSustainingHovers(_hoverControllersBuffer);
        checkSustainingPrimaryHovers(_hoverControllersBuffer);
        checkSustainingContacts(_contactControllersBuffer);
        checkSustainingGrasps(_graspingControllersBuffer);

      }
    }

    #region State-Check Remapping Functions

    private void checkEndingGrasps(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapInteractionObjectStateChecks(
        controllers: interactionControllers,
        stateCheckFunc: (InteractionController maybeReleasingController, out IInteractionBehaviour maybeReleasedObject) => {
          return (maybeReleasingController as IInternalInteractionController).CheckGraspEnd(out maybeReleasedObject);
        },
        actionPerInteractionObject: (releasedObject, releasingIntControllers) => {
          releasedObject.EndGrasp(releasingIntControllers);
        });
    }

    private void checkEndingContacts(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapMultiInteractionObjectStateChecks(
        controllers: interactionControllers,
        multiObjectStateCheckFunc: (InteractionController maybeEndedContactingController, out HashSet<IInteractionBehaviour> endContactedObjects) => {
          return (maybeEndedContactingController as IInternalInteractionController).CheckContactEnd(out endContactedObjects);
        },
        actionPerInteractionObject: (endContactedObject, endContactedIntControllers) => {
          endContactedObject.EndContact(endContactedIntControllers);
        });
    }

    private void checkEndingPrimaryHovers(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapInteractionObjectStateChecks(
        controllers: interactionControllers,
        stateCheckFunc: (InteractionController maybeEndedPrimaryHoveringController, out IInteractionBehaviour endPrimaryHoveredObject) => {
          return (maybeEndedPrimaryHoveringController as IInternalInteractionController).CheckPrimaryHoverEnd(out endPrimaryHoveredObject);
        },
        actionPerInteractionObject: (endPrimaryHoveredObject, noLongerPrimaryHoveringControllers) => {
          endPrimaryHoveredObject.EndPrimaryHover(noLongerPrimaryHoveringControllers);
        });
    }

    private void checkEndingHovers(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapMultiInteractionObjectStateChecks(
        controllers: interactionControllers,
        multiObjectStateCheckFunc: (InteractionController maybeEndedHoveringController, out HashSet<IInteractionBehaviour> endHoveredObjects) => {
          return (maybeEndedHoveringController as IInternalInteractionController).CheckHoverEnd(out endHoveredObjects);
        },
        actionPerInteractionObject: (endHoveredObject, endHoveringIntControllers) => {
          endHoveredObject.EndHover(endHoveringIntControllers);
        });
    }

    private void checkBeginningHovers(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapMultiInteractionObjectStateChecks(
        controllers: interactionControllers,
        multiObjectStateCheckFunc: (InteractionController maybeBeganHoveringController, out HashSet<IInteractionBehaviour> beganHoveredObjects) => {
          return (maybeBeganHoveringController as IInternalInteractionController).CheckHoverBegin(out beganHoveredObjects);
        },
        actionPerInteractionObject: (beganHoveredObject, beganHoveringIntControllers) => {
          beganHoveredObject.BeginHover(beganHoveringIntControllers);
        });
    }

    private void checkBeginningPrimaryHovers(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapInteractionObjectStateChecks(
        controllers: interactionControllers,
        stateCheckFunc: (InteractionController maybeBeganPrimaryHoveringController, out IInteractionBehaviour primaryHoveredObject) => {
          return (maybeBeganPrimaryHoveringController as IInternalInteractionController).CheckPrimaryHoverBegin(out primaryHoveredObject);
        },
        actionPerInteractionObject: (newlyPrimaryHoveredObject, beganPrimaryHoveringControllers) => {
          newlyPrimaryHoveredObject.BeginPrimaryHover(beganPrimaryHoveringControllers);
        });
    }

    private void checkBeginningContacts(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapMultiInteractionObjectStateChecks(
        controllers: interactionControllers,
        multiObjectStateCheckFunc: (InteractionController maybeBeganContactingController, out HashSet<IInteractionBehaviour> beganContactedObjects) => {
          return (maybeBeganContactingController as IInternalInteractionController).CheckContactBegin(out beganContactedObjects);
        },
        actionPerInteractionObject: (beganContactedObject, beganContactingIntControllers) => {
          beganContactedObject.BeginContact(beganContactingIntControllers);
        });
    }

    private void checkBeginningGrasps(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapInteractionObjectStateChecks(
        controllers: interactionControllers,
        stateCheckFunc: (InteractionController maybeBeganGraspingController, out IInteractionBehaviour graspedObject) => {
          return (maybeBeganGraspingController as IInternalInteractionController).CheckGraspBegin(out graspedObject);
        },
        actionPerInteractionObject: (newlyGraspedObject, beganGraspingIntControllers) => {
          newlyGraspedObject.BeginGrasp(beganGraspingIntControllers);
        });
    }

    private void checkSustainingHovers(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapMultiInteractionObjectStateChecks(
        controllers: interactionControllers,
        multiObjectStateCheckFunc: (InteractionController maybeSustainedHoveringController, out HashSet<IInteractionBehaviour> hoveredObjects) => {
          return (maybeSustainedHoveringController as IInternalInteractionController).CheckHoverStay(out hoveredObjects);
        },
        actionPerInteractionObject: (hoveredObject, hoveringIntControllers) => {
          hoveredObject.StayHovered(hoveringIntControllers);
        });
    }

    private void checkSustainingPrimaryHovers(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapInteractionObjectStateChecks(
        controllers: interactionControllers,
        stateCheckFunc: (InteractionController maybeSustainedPrimaryHoveringController, out IInteractionBehaviour primaryHoveredObject) => {
          return (maybeSustainedPrimaryHoveringController as IInternalInteractionController).CheckPrimaryHoverStay(out primaryHoveredObject);
        },
        actionPerInteractionObject: (primaryHoveredObject, primaryHoveringControllers) => {
          primaryHoveredObject.StayPrimaryHovered(primaryHoveringControllers);
        });
    }

    private void checkSustainingContacts(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapMultiInteractionObjectStateChecks(
        controllers: interactionControllers,
        multiObjectStateCheckFunc: (InteractionController maybeSustainedContactingController, out HashSet<IInteractionBehaviour> contactedObjects) => {
          return (maybeSustainedContactingController as IInternalInteractionController).CheckContactStay(out contactedObjects);
        },
        actionPerInteractionObject: (contactedObject, contactingIntControllers) => {
          contactedObject.StayContacted(contactingIntControllers);
        });
    }

    private void checkSustainingGrasps(ReadonlyHashSet<InteractionController> interactionControllers) {
      remapInteractionObjectStateChecks(
        controllers: interactionControllers,
        stateCheckFunc: (InteractionController maybeSustainedGraspingController, out IInteractionBehaviour graspedObject) => {
          return (maybeSustainedGraspingController as IInternalInteractionController).CheckGraspHold(out graspedObject);
        },
        actionPerInteractionObject: (contactedObject, contactingIntControllers) => {
          contactedObject.StayGrasped(contactingIntControllers);
        });
    }

    private delegate bool StateChangeCheckFunc(InteractionController controller, out IInteractionBehaviour obj);
    private delegate bool MultiStateChangeCheckFunc(InteractionController controller, out HashSet<IInteractionBehaviour> objs);

    [ThreadStatic]
    private static Dictionary<IInteractionBehaviour, List<InteractionController>> s_objControllersMap = new Dictionary<IInteractionBehaviour, List<InteractionController>>();

    /// <summary>
    /// Checks object state per-controller, then calls an action per-object with all controller checks that reported back an object.
    /// </summary>
    private void remapInteractionObjectStateChecks(ReadonlyHashSet<InteractionController> controllers,
                                                   StateChangeCheckFunc stateCheckFunc,
                                                   Action<IInteractionBehaviour, List<InteractionController>> actionPerInteractionObject) {

      // Ensure the object->controllers buffer is non-null (ThreadStatic quirk) and clean.
      if (s_objControllersMap == null) s_objControllersMap = new Dictionary<IInteractionBehaviour, List<InteractionController>>();
      s_objControllersMap.Clear();

      // In a nutshell, this remaps methods per-controller that output an interaction object if the controller changed that object's state
      // to methods per-object with all of the controllers for which the check produced a state-change.
      foreach (var controller in controllers) {
        IInteractionBehaviour objectWhoseStateChanged;
        if (stateCheckFunc(controller, out objectWhoseStateChanged)) {
          if (!s_objControllersMap.ContainsKey(objectWhoseStateChanged)) {
            s_objControllersMap[objectWhoseStateChanged] = Pool<List<InteractionController>>.Spawn();
          }
          s_objControllersMap[objectWhoseStateChanged].Add(controller);
        }
      }
      // Finally, iterate through each (object, controllers) pair and call the action for each pair
      foreach (var objControllesPair in s_objControllersMap) {
        actionPerInteractionObject(objControllesPair.Key, objControllesPair.Value);

        // Clear each controllers list and return it to the list pool.
        objControllesPair.Value.Clear();
        Pool<List<InteractionController>>.Recycle(objControllesPair.Value);
      }
    }

    /// <summary>
    /// Checks object state per-controller, then calls an action per-object with all controller checks that reported back objects.
    /// </summary>
    private void remapMultiInteractionObjectStateChecks(ReadonlyHashSet<InteractionController> controllers,
                                                        MultiStateChangeCheckFunc multiObjectStateCheckFunc,
                                                        Action<IInteractionBehaviour, List<InteractionController>> actionPerInteractionObject) {
      // Ensure object<->controllers buffer is non-null (ThreadStatic quirk) and clean.
      if (s_objControllersMap == null) s_objControllersMap = new Dictionary<IInteractionBehaviour, List<InteractionController>>();
      s_objControllersMap.Clear();

      // In a nutshell, this remaps methods per-controller that output multiple interaction objects if the controller changed those objects' states
      // to methods per-object with all of the controllers for which the check produced a state-change.
      foreach (var controller in controllers) {
        HashSet<IInteractionBehaviour> stateChangedObjects;
        if (multiObjectStateCheckFunc(controller, out stateChangedObjects)) {
          foreach (var stateChangedObject in stateChangedObjects) {
            if (!s_objControllersMap.ContainsKey(stateChangedObject)) {
              s_objControllersMap[stateChangedObject] = Pool<List<InteractionController>>.Spawn();
            }
            s_objControllersMap[stateChangedObject].Add(controller);
          }
        }
      }
      // Finally, iterate through each (object, controllers) pair and call the action for each pair
      foreach (var objControllersPair in s_objControllersMap) {
        actionPerInteractionObject(objControllersPair.Key, objControllersPair.Value);

        // Clear each controllers list and return it to the list pool.
        objControllersPair.Value.Clear();
        Pool<List<InteractionController>>.Recycle(objControllersPair.Value);
      }
    }

    #endregion

    #region State Notifications

    // TODO: Delete this whole sction

    //private HashSet<InteractionController> controllerSetBuffer = new HashSet<InteractionController>();

    //void IInternalInteractionManager.NotifyControllerDisabled(InteractionController controller) {
    //  controllerSetBuffer.Clear();
    //  controllerSetBuffer.Add(controller);

    //  checkEndingGrasps(controllerSetBuffer);
    //  checkEndingContacts(controllerSetBuffer);
    //  checkEndingPrimaryHovers(controllerSetBuffer);
    //  checkEndingHovers(controllerSetBuffer);
    //}

    //void IInternalInteractionManager.NotifyHoverDisabled(InteractionController controller) {
    //  controllerSetBuffer.Clear();
    //  controllerSetBuffer.Add(controller);

    //  checkEndingPrimaryHovers(controllerSetBuffer);
    //  checkEndingHovers(controllerSetBuffer);
    //}

    //void IInternalInteractionManager.NotifyContactDisabled(InteractionController controller) {
    //  controllerSetBuffer.Clear();
    //  controllerSetBuffer.Add(controller);

    //  checkEndingContacts(controllerSetBuffer);
    //}

    //void IInternalInteractionManager.NotifyObjectHoverIgnored(IInteractionBehaviour intObj) {
    //  controllerSetBuffer.Clear();

    //  foreach (var controller in interactionControllers) {
    //    if (controller.hoveredObjects.Contains(intObj)) {
    //      (controller as IInternalInteractionController).ClearHoverTrackingForObject(intObj);

    //      controllerSetBuffer.Add(controller);
    //    }
    //  }

    //  checkEndingHovers(controllerSetBuffer);
    //}

    //void IInternalInteractionManager.NotifyObjectPrimaryHoverIgnored(IInteractionBehaviour intObj) {
    //  controllerSetBuffer.Clear();

    //  foreach (var controller in interactionControllers) {
    //    if (controller.primaryHoveredObject == intObj) {
    //      (controller as IInternalInteractionController).ClearPrimaryHoverTrackingForObject(intObj);

    //      controllerSetBuffer.Add(controller);
    //    }
    //  }

    //  checkEndingPrimaryHovers(controllerSetBuffer);
    //}

    //void IInternalInteractionManager.NotifyObjectContactIgnored(IInteractionBehaviour intObj) {
    //  controllerSetBuffer.Clear();

    //  foreach (var controller in interactionControllers) {
    //    if (controller.contactingObjects.Contains(intObj)) {
    //      (controller as IInternalInteractionController).ClearContactTrackingForObject(intObj);

    //      controllerSetBuffer.Add(controller);
    //    }
    //  }

    //  checkEndingContacts(controllerSetBuffer);
    //}

    #endregion

    #endregion

    #region Object Registration

    public void RegisterInteractionBehaviour(IInteractionBehaviour interactionObj) {
      _interactionObjects.Add(interactionObj);
      interactionObjectBodies[interactionObj.rigidbody] = interactionObj;
    }

    /// <summary>
    /// Returns true if the Interaction Behaviour was registered with this manager;
    /// otherwise returns false. The manager is guaranteed not to have the Interaction
    /// Behaviour registered after calling this method.
    /// </summary>
    public bool UnregisterInteractionBehaviour(IInteractionBehaviour interactionObj) {
      bool wasRemovalSuccessful = _interactionObjects.Remove(interactionObj);
      if (wasRemovalSuccessful) {
        foreach (var intController in _interactionControllers) {
          intController.ReleaseObject(interactionObj);

          intController.NotifyObjectUnregistered(interactionObj);
        }
        interactionObjectBodies.Remove(interactionObj.rigidbody);
      }
      return wasRemovalSuccessful;
    }

    public bool IsBehaviourRegistered(IInteractionBehaviour interactionObj) {
      return _interactionObjects.Contains(interactionObj);
    }

    #endregion

    #region Moving Frame of Reference Support

    public bool hasMovingFrameOfReference {
      get {
        return (this.transform.position - _prevPosition).magnitude > 0.0001F
            || Quaternion.Angle(transform.rotation * Quaternion.Inverse(_prevRotation),
                                Quaternion.identity) > 0.01F;
      }
    }

    // Support for a moving frame of reference.
    private Vector3 _prevPosition = Vector3.zero;
    private Quaternion _prevRotation = Quaternion.identity;

    private void updateMovingFrameOfReferenceSupport() {
      _prevPosition = this.transform.position;
      _prevRotation = this.transform.rotation;
    }

    /// <summary>
    /// Transforms a position and rotation ahead by one FixedUpdate based on the prior
    /// motion of the InteractionManager. Use this if your player/InteractionManager's
    /// frame of reference is moving.
    /// </summary>
    public void TransformAheadByFixedUpdate(Vector3 position, Quaternion rotation, out Vector3 newPosition, out Quaternion newRotation) {
      Vector3 worldDisplacement = this.transform.position - _prevPosition;
      Quaternion worldRotation = this.transform.rotation * Quaternion.Inverse(_prevRotation);
      newPosition = ((worldRotation * (position - this.transform.position + worldDisplacement))) + this.transform.position;
      newRotation = worldRotation * rotation;
    }

    #endregion

    #region Internal

    #region Soft Contact Support

    /// <summary>
    /// Stores data for implementing Soft Contact for interaction controllers.
    /// </summary>
    [NonSerialized]
    public List<PhysicsUtility.SoftContact> _softContacts = new List<PhysicsUtility.SoftContact>(80);

    /// <summary>
    /// Stores data for implementing Soft Contact for interaction controllers.
    /// </summary>
    [NonSerialized]
    public Dictionary<Rigidbody, PhysicsUtility.Velocities> _softContactOriginalVelocities = new Dictionary<Rigidbody, PhysicsUtility.Velocities>(5);

    #endregion

    #region Interaction Controllers

    private void refreshInteractionControllers() {
      _interactionControllers.Clear();

      var tempControllers = Pool<List<InteractionController>>.Spawn();
      try {
        this.transform.GetComponentsInChildren<InteractionController>(false, tempControllers);
        foreach (var controller in tempControllers) {
          _interactionControllers.Add(controller);
        }
      }
      finally {
        tempControllers.Clear();
        Pool<List<InteractionController>>.Recycle(tempControllers);
      }
    }

    #endregion

    #region Layers

    protected void generateAutomaticLayers() {
      _interactionLayer = -1;
      _interactionNoContactLayer = -1;
      _contactBoneLayer = -1;
      for (int i = 8; i < 32; i++) {
        string layerName = LayerMask.LayerToName(i);
        if (string.IsNullOrEmpty(layerName)) {
          if (_interactionLayer == -1) {
            _interactionLayer = i;
          } else if (_interactionNoContactLayer == -1) {
            _interactionNoContactLayer = i;
          } else if (_contactBoneLayer == -1) {
            _contactBoneLayer = i;
            break;
          }
        }
      }

      if (_interactionLayer == -1 || _interactionNoContactLayer == -1 || _contactBoneLayer == -1) {
        if (Application.isPlaying) {
          enabled = false;
        }
        Debug.LogError("InteractionManager Could not find enough free layers for "
                     + "auto-setup; manual setup is required.", this.gameObject);
        _autoGenerateLayers = false;
        return;
      }
    }

    private void setupAutomaticCollisionLayers() {
      for (int i = 0; i < 32; i++) {
        // Copy ignore settings from template layer
        bool shouldIgnore = Physics.GetIgnoreLayerCollision(_templateLayer, i);
        Physics.IgnoreLayerCollision(_interactionLayer, i, shouldIgnore);
        Physics.IgnoreLayerCollision(_interactionNoContactLayer, i, shouldIgnore);

        // Set brush layer to collide with nothing
        Physics.IgnoreLayerCollision(_contactBoneLayer, i, true);
      }

      //After copy and set we enable the interaction between the brushes and interaction objects
      Physics.IgnoreLayerCollision(_contactBoneLayer, _interactionLayer, false);
    }

    #endregion

    #endregion

    #region Runtime Gizmos

    public void OnDrawRuntimeGizmos(RuntimeGizmoDrawer drawer) {
      if (_drawControllerRuntimeGizmos) {
        foreach (var controller in _interactionControllers) {
          if (controller != null) {
            controller.OnDrawRuntimeGizmos(drawer);
          }
        }
      }
    }

    #endregion

  }

}
