// this code is responsible for the displaying of a list of placed blocks in the UI/pop-up

using UnityEngine;
using UnityEngine.UI;
using System.Text;
using System.Collections.Generic;

public class BlockListUI : MonoBehaviour                                                        //monobehaviour makes the script attachable to a gameobject in unity
{
    [SerializeField] private GridTileTapHandler tapHandler;                                     //Reference to a script that tracks placed blocks
    [SerializeField] private Text blockListText;                                                //component where the block list will be displayed

    private void OnEnable()                                                                     //When block comes active, add to blocklist
    {
        if (tapHandler != null)
        {
            tapHandler.OnBlockListChanged += PopulateBlockList;
        }
    }

    private void OnDisable()                                                                    //When block comes inactive, take from blocklist
    {
        if (tapHandler != null)
        {
            tapHandler.OnBlockListChanged -= PopulateBlockList;
        }
    }

    public void PopulateBlockList()                                                             //This updates the UI/pop-up text
    {
        if (tapHandler == null || blockListText == null)
        {
            Debug.LogWarning("Missing reference to GridTileTapHandler or blockListText.");      //little debug in case something is not assigned in unity
            return;
        }

        List<BlockData> blocks = tapHandler.GetPlacedBlocks();                                  //gets list of blocks from main code: gridtiletaphandler

        if (blocks == null || blocks.Count == 0)
        {
            blockListText.text = "No blocks placed yet.";                                       //debug
            return;
        }

        StringBuilder sb = new StringBuilder();                                                 //builds the string with the placed blocks
        sb.AppendLine("Placed Blocks:");

        foreach (var block in blocks)                                                           //goes through all the bo
        {
            if (block.id.ToLower().Contains("tile"))                                            // Optional filter: skip tiles if needed (chatgpt)
                continue;

            sb.AppendLine($"{block.id} at ({block.position.x:F1}, {block.position.y:F1}, {block.position.z:F1})");
        }

        blockListText.text = sb.ToString();                                                     //puts information in the UI text
    }
}

