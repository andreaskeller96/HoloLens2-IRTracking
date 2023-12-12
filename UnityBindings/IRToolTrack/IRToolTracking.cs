
using UnityEngine;
using System;
using IRToolTrack;
using System.Linq;


#if ENABLE_WINMD_SUPPORT
using System.Threading.Tasks;
using HL2IRToolTracking;
#endif

public class IRToolTracking : MonoBehaviour
{
#if ENABLE_WINMD_SUPPORT
    HL2IRTracking toolTracking;
#endif
    private bool startToolTracking = false;

    private IRToolController[] tools = null;

    public float[] GetToolTransform(string identifier)
    {
        var toolTransform = Enumerable.Repeat<float>(0, 8).ToArray();
#if ENABLE_WINMD_SUPPORT
        toolTransform = toolTracking.GetToolTransform(identifier);        
#endif
        return toolTransform;

    }

    public Int64 GetTimestamp()
    {
#if ENABLE_WINMD_SUPPORT
        return toolTracking.GetTrackingTimestamp();
#else
        return 0;
#endif
    }

    public void Start()
    {
        //Find Tool Controllers and add them to the tracking
        tools = FindObjectsOfType<IRToolController>();
        StartToolTracking();
    }

    public void StartToolTracking()
    {
            Debug.Log("Start Tracking");
#if ENABLE_WINMD_SUPPORT
        if (!startToolTracking){
            if (toolTracking == null)
            {
                toolTracking = new HL2IRTracking();
            }
            SetReferenceWorldCoordinateSystem();
            toolTracking.RemoveAllToolDefinitions();
            foreach (IRToolController tool in tools)
            {
                int min_visible_spheres = tool.sphere_count;
                if (tool.max_occluded_spheres > 0 && (tool.sphere_count - tool.max_occluded_spheres) >= 3)
                {
                    min_visible_spheres = tool.sphere_count - tool.max_occluded_spheres;
                }
                toolTracking.AddToolDefinition(tool.sphere_count, tool.sphere_positions, tool.sphere_radius, tool.identifier, min_visible_spheres, tool.lowpass_factor_rotation, tool.lowpass_factor_position);
                tool.StartTracking();
            }
            toolTracking.StartToolTracking();
            startToolTracking = true;
        }
#endif
    }

    public void StopToolTracking()
    {
        if (!startToolTracking)
        {
            Debug.Log("Tracking was not started, so cannot stop it");
            return;
        }

#if ENABLE_WINMD_SUPPORT
        var success = toolTracking.StopToolTracking();
        if (!success)
        {
            Debug.Log("Could not stop tracking");
        }
        startToolTracking = false;
        foreach (IRToolController tool in tools)
        {
            tool.StopTracking();
        }
#endif
        Debug.Log("Stopped Tracking");
    }

    private void SetReferenceWorldCoordinateSystem()
    {
        print("Setting World Coordinate");
#if ENABLE_WINMD_SUPPORT
        // Get Unity Origin Coordinate
#if UNITY_2021_2_OR_NEWER
        //var unityWorldOrigin = Microsoft.Windows.Perception.Spatial.SpatialCoordinateSystem
        Windows.Perception.Spatial.SpatialCoordinateSystem unityWorldOrigin = Microsoft.MixedReality.OpenXR.PerceptionInterop.GetSceneCoordinateSystem(UnityEngine.Pose.identity) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#elif UNITY_2020_1_OR_NEWER
        IntPtr WorldOriginPtr = UnityEngine.XR.WindowsMR.WindowsMREnvironment.OriginSpatialCoordinateSystem;
        var unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#else
        IntPtr WorldOriginPtr = UnityEngine.XR.WSA.WorldManager.GetNativeISpatialCoordinateSystemPtr();
        var unityWorldOrigin = Marshal.GetObjectForIUnknown(WorldOriginPtr) as Windows.Perception.Spatial.SpatialCoordinateSystem;
#endif
        // Set Unity Origin Coordinate
        toolTracking.SetReferenceCoordinateSystem(unityWorldOrigin);
#endif


    }

    public void ExitApplication()
    {
        StopToolTracking();
        Application.Quit();
    }
}