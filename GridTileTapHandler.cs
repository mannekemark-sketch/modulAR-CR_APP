//This is the main script that handles placing, demolishing, and saving cubes

using UnityEngine;
using UnityEngine.InputSystem;
using System.Collections.Generic;
using System.IO;
using System;

[Serializable]
public class BlockDataList                                                                          //interacts with blockdatalist for the saving of blockcodes
{
    public List<BlockData> blocks;
}

public enum EditMode                                                                                //creates three edit modes, in unity "None" is the starting position
{
    None,
    Build,
    Demolish
}

public class GridTileTapHandler : MonoBehaviour                                                     //main class handling taps on tiles and cubes, placing/removing cubes, and saving the state
{
    [Header("References")]
    [SerializeField] private Camera arCamera;                                                       //camera that is used for the interaction
    [SerializeField] private BlockListUI blockListUI;                                               //blocklist

    [Header("Settings")]
    public EditMode currentMode = EditMode.None;
    public bool IsInputEnabled { get; set; } = true;
    private PlayerInputActions inputActions;                                                        //unity input system
    private float tapCooldown = 0.2f;                                                               //sets a cooldown when the screen is tapped, to prevent double tap
    private float lastTapTime = 0f;
    private List<BlockData> placedBlocks = new List<BlockData>();                                   //keeps track of placed blocks
    private string saveFilePath;                                                                    //sets filepath for .json file
    private Dictionary<string, GameObject> blockLookup = new Dictionary<string, GameObject>();

    private void Awake()                                                                            //when app is started: initializes input, sets save path, hides all cubes, clears placed blocks, saves initial empty state, and updates UI
    {
        inputActions = new PlayerInputActions();
        saveFilePath = Path.Combine(Application.persistentDataPath, "blocks.json");

        BuildBlockLookup();                                                                         //build dictionary for fast access
        HideAllCubes();
        placedBlocks.Clear();
        SaveBlocksToJson();

        if (blockListUI != null)
            blockListUI.PopulateBlockList();

        Debug.Log("[Awake] GridTileTapHandler initialized.");
    }

    private void OnEnable()                                                                         //subscribes/unsubscribes to the tap input event.
    {
        inputActions.Gameplay.Enable();
        inputActions.Gameplay.Tap.performed += OnTapPerformed;
    }

    private void OnDisable()
    {
        inputActions.Gameplay.Tap.performed -= OnTapPerformed;
        inputActions.Gameplay.Disable();
    }

    public void SetBuildMode() => currentMode = EditMode.Build;                                     //switch between build modes
    public void SetDemolishMode() => currentMode = EditMode.Demolish;

    private void OnTapPerformed(InputAction.CallbackContext context)                                //checks if screen is tapped, cuurent mode, activates input enabled, cooldown
    {
        if (!IsInputEnabled || Time.time - lastTapTime < tapCooldown || currentMode == EditMode.None)
            return;

        lastTapTime = Time.time;
        Vector2 screenPos = context.ReadValue<Vector2>();
        Ray ray = arCamera.ScreenPointToRay(screenPos);

        if (Physics.Raycast(ray, out RaycastHit hit))                                               //raycast
        {
            GameObject target = hit.collider?.gameObject;
            if (target == null) return;

            switch (currentMode)
            {
                case EditMode.Build:                                                                //if buildmode is activated
                    HandleBuildTap(target);
                    break;
                case EditMode.Demolish:                                                             //if demolishmode is activated
                    HandleDemolishTap(target);
                    break;
            }
        }
    }

    private void HandleBuildTap(GameObject target)                                                  //determines where to place the block
    {
        GameObject tile = null;

        if (target.CompareTag("GridTile"))                                                          
            tile = target;                                                                          //if tile tapped, use it
        else if (target.CompareTag("GridCube"))
            tile = target.transform.parent?.gameObject;                                             //if block tapped, use its parent tile

        if (tile == null || !tile.CompareTag("GridTile"))
            return;

        foreach (Transform child in tile.GetComponentsInChildren<Transform>(true))                  //loops through all children of the tile to find the first inactive block
        {
            if (child.CompareTag("GridCube") && !child.gameObject.activeSelf)                       //if inactive cubes exist, make visible
            {
                GameObject cube = child.gameObject;
                cube.SetActive(true);

                if (!placedBlocks.Exists(b => b.id == cube.name))
                {
                    placedBlocks.Add(new BlockData(                                                 //when cube is placed, add blockdata
                        cube.name,
                        tile.name,
                        cube.transform.position,
                        GetStackHeightAt(cube.transform.position.x, cube.transform.position.z)      //stackheight is currently not used, could help in future
                    ));
                    Debug.Log($"[Build] Placed cube: {cube.name} on tile: {tile.name}");            //Lil debug
                }

                OnBlockListChanged?.Invoke();                                                       //add to .json file
                SaveBlocksToJson();
                return;
            }
        }
    }

    private void HandleDemolishTap(GameObject target)                                                   //demolishes the top block at a location
    {
        if (!target.CompareTag("GridCube")) return;

        GameObject topCube = GetTopActiveCubeAt(target.transform.position.x, target.transform.position.z); //finds top block
        if (topCube != target) return;

        topCube.SetActive(false);                                                                       //disable visibility block
        placedBlocks.RemoveAll(b => b.id == topCube.name);

        Debug.Log($"[Demolish] Removed cube: {topCube.name}");                                          //debug

        OnBlockListChanged?.Invoke();
        SaveBlocksToJson();                                                                             //take block from .json file
    }

    private List<GameObject> GetCubesAt(float x, float z)                                               //finds all blocks at x,z position
    {
        GameObject[] allCubes = GameObject.FindGameObjectsWithTag("GridCube");
        List<GameObject> cubes = new List<GameObject>();
        foreach (var c in allCubes)
            if (Mathf.Approximately(c.transform.position.x, x) && Mathf.Approximately(c.transform.position.z, z))
                cubes.Add(c);
        return cubes;
    }

    private GameObject GetTopActiveCubeAt(float x, float z)                                             //gets top block that is active
    {
        List<GameObject> cubes = GetCubesAt(x, z);
        GameObject topCube = null;
        float highestY = float.MinValue;
        foreach (var c in cubes)
        {
            if (c.activeSelf && c.transform.position.y > highestY)
            {
                highestY = c.transform.position.y;
                topCube = c;
            }
        }
        return topCube;
    }

    private int GetStackHeightAt(float x, float z)                                                      //counts how many blocks are stacked at position (height)
    {
        int height = 0;
        foreach (var c in GetCubesAt(x, z))
            if (c.activeSelf) height++;
        return height;
    }

    private void HideAllCubes()                                                                         //hides all blocks on app initiation 
    {
        GameObject[] cubes = GameObject.FindGameObjectsWithTag("GridCube");
        foreach (var c in cubes)
            c.SetActive(false);

        Debug.Log("[HideAllCubes] All cubes hidden");
    }

    private void BuildBlockLookup()                                                                     //creates dictionary of all blocks gameobjects, for prefab variants
    {
        blockLookup.Clear();
        foreach (GameObject cube in GameObject.FindGameObjectsWithTag("GridCube"))
        {
            if (!blockLookup.ContainsKey(cube.name))
                blockLookup[cube.name] = cube;
        }
        Debug.Log($"[BuildBlockLookup] Found {blockLookup.Count} cubes in scene.");
    }

    private GameObject FindCubeByName(string name)                                                      //looks for blocks in dictionary
    {
        return blockLookup.TryGetValue(name, out GameObject cube) ? cube : null;
    }


    public void BuildBlocksByNames(List<string> blockIDs)                                               //called by variantselectionUI script, hides all existing blocks
    {
        HideAllCubes();
        placedBlocks.Clear();                                                                           //hide active blocks

        Debug.Log("[BuildBlocksByNames] Building variant with blocks:");
        foreach (var id in blockIDs)
            Debug.Log(" - " + id);

        foreach (string id in blockIDs)
        {
            GameObject cube = FindCubeByName(id);                                                       //finds blocks in dictionary, by their name
            if (cube == null)
            {
                Debug.LogWarning("[BuildBlocksByNames] Block not found: " + id);
                continue;
            }

            HandleBuildTap(cube);
        }

        Debug.Log($"[BuildBlocksByNames] Built variant with {blockIDs.Count} requested blocks. Total placedBlocks: {placedBlocks.Count}");

        OnBlockListChanged?.Invoke();
        SaveBlocksToJson();                                                                             //saves newly placed blocks to json
    }

    public List<BlockData> GetPlacedBlocks() => placedBlocks;

    private void SaveBlocksToJson()
    {
        BlockDataList wrapper = new BlockDataList { blocks = placedBlocks };                            //functions that converts placed blockcodes into the .json file
        string json = JsonUtility.ToJson(wrapper, true);
        File.WriteAllText(saveFilePath, json);

        Debug.Log($"[SaveBlocksToJson] Saved {placedBlocks.Count} blocks to JSON at {saveFilePath}");
    }

    public event Action OnBlockListChanged;
}
