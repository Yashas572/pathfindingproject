using UnityEngine; // Core Unity engine functions
using System.Collections.Generic; // For using List<T>
using TMPro; // For showing UI text (TextMeshPro)


public class SteeringBehavior : MonoBehaviour
{
   public Vector3 target; // Current point the car is seeking
   public KinematicBehavior kinematic; // Script controlling movement and rotation
   public List<Vector3> path; // List of waypoints for path-following
   public TextMeshProUGUI label; // Debug label to show distance or status
   public float arrival_distance; // How close to a point counts as "arrived"


   private bool isPathFollowing = false; // Are we currently in path-following mode?


   void Start()
   {
       kinematic = GetComponent<KinematicBehavior>(); // Grab the movement component
       target = transform.position; // Default target is current position
       path = null; // No path yet
       isPathFollowing = false; // Start in single-target mode
       EventBus.OnSetMap += SetMap; // Listen for map reset
   }


   void Update()
   {
       // ---------------- PATH FOLLOWING ----------------
       if (isPathFollowing && path != null && path.Count > 0)
       {
           Vector3 toTarget = target - transform.position; // Vector to current waypoint
           toTarget.y = 0f; // Ignore vertical difference
           float distance = toTarget.magnitude; // Distance to current waypoint


           // If close to the waypoint, move to the next one early
           if (distance < arrival_distance * 1.2f)
           {
               path.RemoveAt(0); // Remove the reached waypoint


               if (path.Count > 0)
               {
                   target = path[0]; // Set next target
                   EventBus.ShowTarget(target); // Visualize it (optional)
               }
               else
               {
                   path = null; // Clear path
                   isPathFollowing = false; // Back to single-target mode
               }
           }
       }


       // ---------------- TARGET SEEKING ----------------

       if (!isPathFollowing)
       {
           Vector3 toTarget = target - transform.position; // Direction to target
           toTarget.y = 0f;


           float distance = toTarget.magnitude; // Distance to target


           if (label != null)
               label.text = "Distance to target: " + distance.ToString("F2"); // Debug text


           // Stop completely if we're at target and nearly stopped
           if (distance < arrival_distance && Mathf.Abs(kinematic.speed) < 0.5f)
           {
               kinematic.speed = 0f;
               kinematic.rotational_velocity = 0f;
               kinematic.SetDesiredSpeed(0f);
               kinematic.SetDesiredRotationalVelocity(0f);


               if (label != null)
                   label.text = "Arrived at target!";
               return; // Done for this frame
           }
       }


       // ---------------- ROTATION ----------------
       Vector3 direction = target - transform.position;
       direction.y = 0f;


       // Avoid division by zero or invalid angles
       if (direction.sqrMagnitude < 0.001f) return;


       Vector3 forward = transform.forward; // Current facing direction
       float angle = Vector3.SignedAngle(forward, direction.normalized, Vector3.up); // Signed angle between forward and target


       float maxRot = kinematic.GetMaxRotationalVelocity(); // Max turning speed
       float clampedAngle;

       if (Mathf.Abs(angle) < 1f) {
        clampedAngle = 0f;
       }
       else {
        float curvedAngle = angle * Mathf.Abs(angle) * 0.1f;
        clampedAngle = Mathf.Clamp(curvedAngle, -maxRot, maxRot); // Clamp to max turning sped
       }

       kinematic.SetDesiredRotationalVelocity(clampedAngle); // Turn towards target


       // ---------------- SPEED CONTROL ----------------
       float distanceToTarget = direction.magnitude;
       float maxSpeed = kinematic.GetMaxSpeed();
       float slowRadius = arrival_distance * 3f;


       float targetSpeed = maxSpeed; // Default to max speed


       if (isPathFollowing && path != null && path.Count > 0)
       {
           // Look ahead to next direction (for smoother turning)
           Vector3 nextDir = path[0] - target;
           nextDir.y = 0f;


           if (nextDir.sqrMagnitude > 0.01f)
           {
               float turnAngle = Vector3.Angle(direction, nextDir); // How sharp is the next turn?
               float slowdownFactor = Mathf.Clamp01(turnAngle / 180f); // Bigger angle = more speed
               targetSpeed *= Mathf.Lerp(0.3f, 1f, slowdownFactor); // Slow down for sharper turns
           }


           if (distanceToTarget < slowRadius)
           {
               targetSpeed *= distanceToTarget / slowRadius; // Ease into waypoint
           }
       }
       else
       {
           if (distanceToTarget < slowRadius)
           {
               targetSpeed = maxSpeed * (distanceToTarget / slowRadius); // Slow for final target
           }
       }


       kinematic.SetDesiredSpeed(targetSpeed); // Move forward
   }


   // Called when a single target is clicked (left-click)
   public void SetTarget(Vector3 target)
   {
       this.target = target; // Set new target
       this.path = null; // Clear any path
       this.isPathFollowing = false; // Single-target mode
       EventBus.ShowTarget(target); // Visualize it
   }


   // Called when a path is drawn (right-clicks)
    public void SetPath(List<Vector3> path)
    {   
        // Prevent errors by checking if the path is null or empty
        if (path == null || path.Count == 0)
        {
            return;
        }

        // Copy the given path to the internal list
        this.path = new List<Vector3>(path);

        // Set the first point in the path as the initial target
        this.target = this.path[0];

        // Show the target on screen (optional for debugging)
        EventBus.ShowTarget(target);

        // Enable path-following mode
        this.isPathFollowing = true;
    }


   // Called when the map is reset
   public void SetMap(List<Wall> outline)
   {
       this.path = null; // Clear everything
       this.target = transform.position;
       this.isPathFollowing = false;
   }
}

