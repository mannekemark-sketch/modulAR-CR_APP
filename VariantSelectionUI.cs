using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class VariantSelectionUI : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private GridTileTapHandler gridHandler;
    [SerializeField] private UIController uiController;

    [SerializeField] private Button variant1Button;
    [SerializeField] private Button variant2Button;
    [SerializeField] private Button variant3Button;
    [SerializeField] private Button variant4Button;

    private readonly Dictionary<int, List<string>> variants = new Dictionary<int, List<string>>()       //creates a string with the blockcodes from the premade structure
    {
        { 1, new List<string> { "A-05-1", "B-05-1", "C-05-1", "D-05-1", "A-02-1", "B-02-1", "E-01-1", "E-01-2", "E-01-3", "E-02-1", "E-02-2", "E-02-3" } }, 
        { 2, new List<string> { "A-05-1", "A-05-2", "A-05-3", "E-05-1", "E-05-2", "E-05-3", "B-02-1", "B-02-2", "C-02-1", "C-02-2" } },
        { 3, new List<string> { "A-03-1", "A-04-1", "C-01-1", "C-03-1", "C-03-2", "C-04-1", "C-04-2", "E-01-1", "E-03-1", "E-03-2", "E-04-1", "E-04-2" } },
        { 4, new List<string> { "B-02-1", "B-02-2", "B-03-1", "B-03-2", "C-02-1", "C-02-2", "C-03-1", "C-03-2", "D-02-1", "D-02-2", "D-03-1", "D-03-2" } }
    };

    private void Start()                                                                                //when button is clicked, corresponding variant is activated           
    {
        variant1Button.onClick.AddListener(() => SelectVariant(1));
        variant2Button.onClick.AddListener(() => SelectVariant(2));
        variant3Button.onClick.AddListener(() => SelectVariant(3));
        variant4Button.onClick.AddListener(() => SelectVariant(4));
    }

    private void SelectVariant(int id)                                                                  //check is variant exists -> debug
    {
        if (!variants.ContainsKey(id))
        {
            Debug.LogWarning("Variant not found: " + id);
            return;
        }

        gridHandler.BuildBlocksByNames(variants[id]);                                                   //activates a part of the main script that hides all blocks

        if (uiController != null)
            uiController.CloseVariantPopup();                                                           //closes pop-up when variant is selected
    }
}
