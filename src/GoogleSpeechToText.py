#!/usr/bin/env python3
import sys
import nltk
import aiml

import speech_recognition as sr
import time
import gtts
from gtts import gTTS
from pygame import mixer




def say(string):
    mixer.init()
    tts = gTTS(text=string, lang='en')
    tts.save("response.mp3")
    mixer.music.load('response.mp3')
    mixer.music.play()

def speech_rec():
    while True:

        r = sr.Recognizer()
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=1)
            print("Say something!")
            audio = r.listen(source, phrase_time_limit=5)
            try:
                response = r.recognize_google(audio, language="en-US")

                response = response.lower()
 
                print("Did you say "+response)
                print(response)
                if response=="yes" or response=="no":
                    return str(response)
            except sr.RequestError as e:
                print("Could not request results; {0}".format(e))
                say("Can you repeat again!")
         
            except sr.UnknownValueError:
                print("unknown error occured")
                say("Can you repeat again!")



