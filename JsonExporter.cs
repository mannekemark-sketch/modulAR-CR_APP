using UnityEngine;
using System.Diagnostics;
using System.IO;
using System.Text;

using UDebug = UnityEngine.Debug;                                                                           // Alias to avoid ambiguity`-chatgpt solution

public class JsonExporter : MonoBehaviour
{
    [SerializeField] private GridTileTapHandler tapHandler;                                                 

    public void ExportAndSendToROS()
    {
        if (tapHandler == null)
        {
            UDebug.LogWarning("GridTileTapHandler reference is missing.");                                  //makes sure main script is attached -> debug
            return;
        }

        var placedBlocks = tapHandler.GetPlacedBlocks();                                                    //get the placed block codes
        StringBuilder psScript = new StringBuilder();

        string sshUser = "mirte";
        string sshHost = "mirte-676726.local";

        foreach (var block in placedBlocks)
        {
            if (!string.IsNullOrEmpty(block.id))
            {
                psScript.AppendLine($"ssh {sshUser}@{sshHost} ./publish.sh \"{block.id}\"");                // Add a line for each block
            }
        }

        string tempScriptPath = Path.Combine(Application.persistentDataPath, "SendBlocks.ps1");             //save temporary PS1 script
        File.WriteAllText(tempScriptPath, psScript.ToString());

        ProcessStartInfo psi = new ProcessStartInfo                                                         // Open PowerShell with the script ready to run and window stays open
        {
            FileName = "powershell.exe",
            Arguments = $"-NoProfile -ExecutionPolicy Bypass -NoExit -File \"{tempScriptPath}\"",
            UseShellExecute = true
        };

        Process.Start(psi);
        UDebug.Log("PowerShell opened with SSH commands ready. Enter your password when prompted by SSH.");
    }
}
