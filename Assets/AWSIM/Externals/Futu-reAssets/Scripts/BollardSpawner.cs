using System.Collections;
using System.Collections.Generic;
using AWSIM;
using UnityEngine;
using UnityEngine.UI;
using System;


public class BollardSpawner : MonoBehaviour
{
    public GameObject bollardObject; // The object that will be spawned


    // [System.Serializable]
    public class BollardLine {
        [SerializeField] public Vector3 start;
        [SerializeField] public Vector3 end;
    }
    /* [SerializeField] */
    //public BollardLine bollardLineCrossroadEast, bollardLineCrossroadWest, bollardLineCourtyardSouth, bollardLineCourtyardNorth;
    
    public GameObject bollardCrossroadEast0;
    public GameObject bollardCrossroadEast1;
    public GameObject bollardCrossroadWest0;
    public GameObject bollardCrossroadWest1;
    public GameObject bollardCourtyardSouth0;
    public GameObject bollardCourtyardSouth1;
    public GameObject bollardCourtyardNorth0;
    public GameObject bollardCourtyardNorth1;

    public InputField inputFieldForPitchValue;
    public InputField inputFieldForOffsetValue;
    public Slider sliderForPitch;
    public Slider sliderForOffset;

    public GameObject originalBollard1;
    public GameObject originalBollard2;
    public GameObject originalBollard3;
    public GameObject originalBollard4;
    public GameObject originalBollard5;
    public GameObject originalBollard6;
    public GameObject originalBollard7;
    public GameObject originalBollard8;
    public GameObject originalBollard9;
    public GameObject originalBollard10;
    public GameObject originalBollard11;
    public GameObject originalBollard12;

    private class BollardLinePrivate
    {
        private GameObject bollardObject;
        public BollardLine bollardLine;
        public Vector3 dir;
        public float length;
        public uint numOfBollardsMax;
        public GameObject[] bollards;
        public BollardLinePrivate(GameObject _bollardObject, BollardLine _bollardLine, uint _num)
        {
            bollardObject = _bollardObject;
            bollardLine = _bollardLine;
            numOfBollardsMax = _num;
            dir = (bollardLine.end - bollardLine.start).normalized;
            length = (bollardLine.end - bollardLine.start).magnitude;
            Debug.Log("dir:" + dir + " length:" + length);
            bollards = new GameObject[numOfBollardsMax];
        }
        public void SetOffsetAndPitch(float offset, float pitch)
        {
            // destroy existing objects
            for (int i=0; i < numOfBollardsMax; i++) {
                if (bollards[i]) {
                    Destroy(bollards[i]);
                    bollards[i] = null;
                    //Debug.Log("i:" + i + " destroyed a bollardObject.");
                }
            }
            // create 
            for (int i = 0; i < numOfBollardsMax; i++)
            {
                float distance;
                if (i < numOfBollardsMax/2) {
                    distance = offset + length/2.0f + i*pitch;
                } else {
                    distance = offset + length/2.0f + (i-numOfBollardsMax)*pitch;
                }
                if (length < distance || distance < 0) {
                    Debug.Log("i:" + i + " skipped (length:" + length + ", distance:" + distance);
                    continue;
                }
                Vector3 position = bollardLine.start + distance * dir;
                bollards[i] = Instantiate(bollardObject, position, Quaternion.identity);
                //bollards[i].transform.localScale = new Vector3(1.25f, 1.0f, 1.25f);      // bolder objects are used with this scale (50cm -> 62.5cm)
                bollards[i].SetActive(true);
                //Debug.Log("i:" + i + " instantiated a bollardObject at " + position);
            }
        }
        public void SetActive(bool _active)
        {
            for (int i = 0; i < numOfBollardsMax; i++)
            {
                if (bollards[i]) {
                    bollards[i].SetActive(_active);
                }
            }
        }
    }
    public enum BollardLineIndex
    {
        IntersectionEast,
        IntersectionWest,
        CourtyardSouth,
        CourtyardNorth,
        NumOfElements
    };

    private bool customizeBollard = false;
    private BollardLine[] bollardLines;
    private BollardLinePrivate[] bollardLinesPrivate;
    bool isModified = true;
    float offset = 0.0f; // [m]
    float pitch = 2.0f; // [m]

    void Awake()
    {
        bollardLines = new BollardLine[(uint)BollardLineIndex.NumOfElements];
        for (int i = 0; i < (uint)BollardLineIndex.NumOfElements; i++)
        {
            bollardLines[i] = new BollardLine();
        }
        bollardLines[(uint)BollardLineIndex.IntersectionEast].start = bollardCrossroadEast0.transform.position;
        bollardLines[(uint)BollardLineIndex.IntersectionEast].end   = bollardCrossroadEast1.transform.position;
        bollardLines[(uint)BollardLineIndex.IntersectionWest].start = bollardCrossroadWest0.transform.position;
        bollardLines[(uint)BollardLineIndex.IntersectionWest].end   = bollardCrossroadWest1.transform.position;
        bollardLines[(uint)BollardLineIndex.CourtyardSouth  ].start = bollardCourtyardSouth0.transform.position;
        bollardLines[(uint)BollardLineIndex.CourtyardSouth  ].end   = bollardCourtyardSouth1.transform.position;
        bollardLines[(uint)BollardLineIndex.CourtyardNorth  ].start = bollardCourtyardNorth0.transform.position;
        bollardLines[(uint)BollardLineIndex.CourtyardNorth  ].end   = bollardCourtyardNorth1.transform.position;
        //bollardLines[(uint)BollardLineIndex.CourtyardNorth] = bollardCourtyardNorth;
        Debug.Log("bollardCrossroadEast0.transform.position: " + bollardCrossroadEast0.transform.position);
        Debug.Log("bollardCrossroadEast1.transform.position: " + bollardCrossroadEast1.transform.position);
        Debug.Log("bollardCourtyardNorth1.transform.position: " + bollardCourtyardNorth1.transform.position);

        bollardLinesPrivate = new BollardLinePrivate[(uint)BollardLineIndex.NumOfElements];
        for (int i = 0; i < (uint)BollardLineIndex.NumOfElements; i++)
        {
            bollardLinesPrivate[i] = new BollardLinePrivate(bollardObject, bollardLines[i], 32);
        }
    }

    void Start()
    {
        inputFieldForPitchValue.text = pitch.ToString();
        sliderForPitch.value = pitch;
        inputFieldForOffsetValue.text = offset.ToString();
        sliderForOffset.value = offset;
    }

    void Update()
    {
        if (bollardObject == null)
        {
            return;
        }

        if (!isModified)
        {
            return;
        }
        originalBollard1.SetActive(!customizeBollard);
        originalBollard2.SetActive(!customizeBollard);
        originalBollard3.SetActive(!customizeBollard);
        originalBollard4.SetActive(!customizeBollard);
        originalBollard5.SetActive(!customizeBollard);
        originalBollard6.SetActive(!customizeBollard);
        originalBollard7.SetActive(!customizeBollard);
        originalBollard8.SetActive(!customizeBollard);
        originalBollard9.SetActive(!customizeBollard);
        originalBollard10.SetActive(!customizeBollard);
        originalBollard11.SetActive(!customizeBollard);
        originalBollard12.SetActive(!customizeBollard);
        if (customizeBollard) {
            Debug.Log("Offset or pitch is modified. Modify bollard positions..");
            for (uint i = 0; i < (uint)BollardLineIndex.NumOfElements; i++)
            {
                bollardLinesPrivate[i].SetOffsetAndPitch(offset, pitch);
            }
        } else {
            for (uint i=0; i < (uint)BollardLineIndex.NumOfElements; i++)
            {
                bollardLinesPrivate[i].SetActive(false);
            }
        }
        isModified = false;
        Debug.Log("Modified.");
    }

    public void OnPitchChange(float _pitch)
    {
        Debug.Log("_pitch:" + _pitch);
        if (pitch != _pitch)
        {
            isModified = true;
            pitch = _pitch;
            inputFieldForPitchValue.text = pitch.ToString();
        }
    }

    public void OnOffsetChange(float _offset)
    {
        Debug.Log("_offset:" + _offset);
        if (offset != _offset)
        {
            isModified = true;
            offset = _offset;
            inputFieldForOffsetValue.text = offset.ToString();
        }
    }

    public void OnPitchChangeFromInputField(string _pitchString)
    {
        float newPitch = Mathf.Clamp(float.Parse(_pitchString), 1.0f, 10.0f);
        if (newPitch != pitch) {
            isModified = true;
            pitch = newPitch;
            sliderForPitch.value = pitch;
        }
    }

    public void OnOffsetChangeFromInputField(string _offsetString)
    {
        float newOffset = Mathf.Clamp(float.Parse(_offsetString), -5.0f, 5.0f);
        if (newOffset != offset) {
            isModified = true;
            offset = newOffset;
            sliderForOffset.value = offset;
        }
    }

    public void OnChangeLayoutCustomize(bool _customize)
    {
        if (customizeBollard != _customize) {
            isModified = true;
            customizeBollard = _customize;
        }
    }
}

