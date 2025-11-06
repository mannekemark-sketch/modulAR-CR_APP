// This code manages the user interface for building, demolishing, exporting, and selecting variants.

using UnityEngine;
using UnityEngine.UI;

public class UIController : MonoBehaviour
{
    [Header("Main Buttons")]
    [SerializeField] private Button buildButton;
    [SerializeField] private Button demolishButton;
    [SerializeField] private Button exportButton;
    [SerializeField] private Button variantButton; 

    [Header("UI Panels")]
    [SerializeField] private GameObject exportPopupPanel;
    [SerializeField] private GameObject variantPopupPanel;

    [Header("References")]
    [SerializeField] private BlockListUI blockListUI;
    [SerializeField] private GridTileTapHandler tapHandler;

    private void Start()                                                                        //buttons in the main view
    {
        if (buildButton) buildButton.onClick.AddListener(OnBuildClicked);
        if (demolishButton) demolishButton.onClick.AddListener(OnDemolishClicked);
        if (exportButton) exportButton.onClick.AddListener(OnExportClicked);
        if (variantButton) variantButton.onClick.AddListener(OnVariantClicked);
    }

    private void OnDestroy()                                                                    //prevents "false" clikcs
    {   
        if (buildButton) buildButton.onClick.RemoveListener(OnBuildClicked);
        if (demolishButton) demolishButton.onClick.RemoveListener(OnDemolishClicked);
        if (exportButton) exportButton.onClick.RemoveListener(OnExportClicked);
        if (variantButton) variantButton.onClick.RemoveListener(OnVariantClicked);
    }

    // ---------------- Variant Button ----------------
    private void OnVariantClicked()
    {
        if (variantPopupPanel) variantPopupPanel.SetActive(true);                               //opens pop-up

        
        ToggleMainButtons(false);                                                               // Hide main buttons while popup is open

        if (tapHandler) tapHandler.IsInputEnabled = false;
    }

    public void CloseVariantPopup()                                                             //close pop-up
    {
        if (variantPopupPanel) variantPopupPanel.SetActive(false);                              //restore main UI
        ToggleMainButtons(true);
        if (tapHandler) tapHandler.IsInputEnabled = true;
    }

    // ---------------- Build / Demolish / Export ----------------
    private void OnBuildClicked()
    {
        tapHandler.SetBuildMode();
        UpdateButtonColors(EditMode.Build);                                                     //set build mode
    }

    private void OnDemolishClicked()                                    
    {
        tapHandler.SetDemolishMode();
        UpdateButtonColors(EditMode.Demolish);                                                  //set demolish mode
    }

    private void OnExportClicked()
    {
        if (exportPopupPanel) exportPopupPanel.SetActive(true);                                 //opens export pop-up
        ToggleMainButtons(false);                                                               //hides main buttons
        if (tapHandler) tapHandler.IsInputEnabled = false;                                      //disables placement of blocks

        if (blockListUI) blockListUI.PopulateBlockList();                                       //populate blocklist
    }

    public void CloseExportPopup()
    {
        if (exportPopupPanel) exportPopupPanel.SetActive(false);
        ToggleMainButtons(true);                                                                //restore main UI
        if (tapHandler) tapHandler.IsInputEnabled = true;
    }

    // ---------------- Utility ----------------
    private void ToggleMainButtons(bool state)                                                  //hides/restores main buttons, used above
    {
        if (buildButton) buildButton.gameObject.SetActive(state);
        if (demolishButton) demolishButton.gameObject.SetActive(state);
        if (exportButton) exportButton.gameObject.SetActive(state);
        if (variantButton) variantButton.gameObject.SetActive(state);
    }

    private void UpdateButtonColors(EditMode mode)                                              //colors buttons when pressed
    {
        Color active = Color.green;                                     
        Color inactive = Color.red;

        if (buildButton)
            buildButton.GetComponent<Image>().color = mode == EditMode.Build ? active : inactive;

        if (demolishButton)
            demolishButton.GetComponent<Image>().color = mode == EditMode.Demolish ? active : inactive;
    }

    public void OpenVariantPopup()
    {
        if (variantPopupPanel != null)
            variantPopupPanel.SetActive(true);

        Debug.Log("[UIController] Variant popup opened.");
    }
}
