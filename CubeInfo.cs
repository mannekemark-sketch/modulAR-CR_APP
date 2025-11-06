//This script makes sure that the cubes keep their own name

using UnityEngine;

[ExecuteAlways] // Updates in editor and runtime
public class CubeInfo : MonoBehaviour
{
    [SerializeField] private string cubeCode;   // Visible in Inspector

    public string CubeCode => cubeCode; // Read-only access from other scripts

    private void Awake()
    {
        UpdateCubeCode();
    }

    private void OnValidate()
    {
        UpdateCubeCode();
    }

    private void UpdateCubeCode()
    {
        cubeCode = gameObject.name;
    }
}
