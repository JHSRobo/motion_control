**MIDI\_CONTROLLER** 


Created by David Janssen

This is a program to use a Korg nanoKontrol2 MIDI controller with updating horizontal, vertical and angular sensitivities.
**KORG nanoKontrol2** 


**Uses MIDIUtil:**

Install MIDIUtil


~~~bash
pip install MIDIUtil
~~~
*Clone to git: [https://github.com/MarkCWirt/MIDIUtil.git](https://github.com/MarkCWirt/MIDIUtil.git)*
~~~bash
git clone https://github.com/MarkCWirt/MIDIUtil.git 
~~~

Install rtmidi

~~~bash  
  pip install python-rtmidi  
~~~

Button mapping for the rest of the nanoKontrol2 controller, including those not used for sensitivities:

~~~bash  
buttons \= {  
    0: "Slider 1",  
    1: "Slider 2",  
    2: "Slider 3",  
    3: "Slider 4",  
    4: "Slider 5",  
    5: "Slider 6",  
    6: "Slider 7",  
    7: "Slider 8",  
    16: "Knob 1",  
    17: "Knob 2",  
    18: "Knob 3",  
    19: "Knob 4",  
    20: "Knob 5",  
    21: "Knob 6",  
    22: "Knob 7",  
    23: "Knob 8",  
    32: "S button 1",  
    33: "S button 2",  
    34: "S button 3",  
    35: "S button 4",  
    36: "S button 5",  
    37: "S button 6",  
    38: "S button 7",  
    39: "S button 8",  
    48: "M button 1",  
    49: "M button 2",  
    50: "M button 3",  
    51: "M button 4",  
    52: "M button 5",  
    53: "M button 6",  
    54: "M button 7",  
    55: "M button 8",  
    64: "R button 1",  
    65: "R button 2",  
    66: "R button 3",  
    67: "R button 4",  
    68: "R button 5",  
    69: "R button 6",  
    70: "R button 7",  
    71: "R button 8",  
    58: "Track left",  
    59: "track right",  
    46: "Cycle",  
    60: "Set",  
    61: "Marker left",  
    62: "Marker right",  
    43: "\<\< fast backword???",  
    44: "\>\> fast forwards",  
    42: "stop/square",  
    41: "play/triangle",  
    45: "record/circle",  
}

~~~
