// this code creates a data container for every block in Unity

using UnityEngine;
[System.Serializable]
public class BlockData
{
    public string id;
    public string tileCode;
    public Vector3 position;
    public int stackIndex;

    public BlockData(string id, string tileCode, Vector3 position, int stackIndex)
    {
        this.id = id;
        this.tileCode = tileCode;
        this.position = position;
        this.stackIndex = stackIndex;
    }
}
