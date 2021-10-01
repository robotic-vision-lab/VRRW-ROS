using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class UpdateSliderText : MonoBehaviour
{
    public Text valueDisplay;
    public Slider slider;

    // Start is called before the first frame update
    void Start()
    {
        slider = gameObject.GetComponent<Slider>();
        valueDisplay = slider.transform.Find("ValueDisplay").GetComponent<Text>();
        valueDisplay.text = makeTempString(slider.value);

        slider.onValueChanged.AddListener(delegate {UpdateText();});
    }

    // Update is called once per frame
    void Update()
    {
    }

    void UpdateText()
    {
        valueDisplay.text = makeTempString(slider.value);
    }

    string makeTempString(float input)
    {
        return input.ToString() + "\u00B0";
    }
}
