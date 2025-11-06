//This script handles fading out an intro panel and enabling a button panel afterward in Unity

using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class IntroOverlayFade : MonoBehaviour
{
    [Header("Fade Settings")]
    public float fadeDuration = 1f;                                                             // seconds to fade out

    [Header("References")]
    public GameObject Canvas_Button;                                                            // assign Canvas_Buttonpanel here

    private CanvasGroup canvasGroup;

    void Awake()
    {
        canvasGroup = GetComponent<CanvasGroup>();
        if (canvasGroup == null)
        {
            Debug.LogError("❌ CanvasGroup missing! Add one to " + gameObject.name);            //check if canvas is attached, otherwise doesnt launch the script -> debug
            this.enabled = false;
            return;
        }

        canvasGroup.alpha = 1f;                                                                 //start fully visible
        canvasGroup.interactable = true;                                                        //canvasgroup set to interactable
        canvasGroup.blocksRaycasts = true;                                                      //blocks raycast to objects behind it
                                                                                                
        if (Canvas_Button)
            Canvas_Button.SetActive(false);                                                     //ensure Canvas_Button starts hidden
    }

    public void ContinueButtonPressed()                                                         //call this when the button is pressed
    {
        StartCoroutine(FadeOut());
    }

    private IEnumerator FadeOut()
    {
        float startAlpha = canvasGroup.alpha;
        float elapsed = 0f;                                                                     //sets canvas to invisible

        while (elapsed < fadeDuration)
        {
            elapsed += Time.deltaTime;
            canvasGroup.alpha = Mathf.Lerp(startAlpha, 0f, elapsed / fadeDuration);             //gradually fades out
            yield return null;
        }

        canvasGroup.alpha = 0f;
        canvasGroup.interactable = false;
        canvasGroup.blocksRaycasts = false;                                                      
        gameObject.SetActive(false);                                                             //makes sure the canvas isnt activated but turning it off

        if (Canvas_Button)                                                                      // Enable Canvas_Button after fade
            Canvas_Button.SetActive(true);
    }
}
