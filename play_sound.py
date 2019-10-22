'''
Reference: https://pythonprogramminglanguage.com/text-to-speech/
           https://pyttsx.readthedocs.io/en/v1.0/engine.html#pyttsx.voice.Voice

'''
import pyttsx
import time

test_voice = False

def play_sound():
    engine = pyttsx.init()
    engine.setProperty('rate', 150)
    engine.setProperty('voice', b'english-us')

    engine.runAndWait()

    rate = engine.getProperty('rate')
    volume = engine.getProperty('volume')
    voice = engine.getProperty('voice')         # get id
    voices = engine.getProperty('voices')
    engine.runAndWait()                         # GetProperty isn't updated until after you've used the runAndWait() method
                                                # issue on https://github.com/nateshmbhat/pyttsx3/issues/59

    print("Rate: {} Volume: {} Voice: {}".format(rate,volume,voice))

    if test_voice:
        # Test which voices is better
        for voice in voices:
            print("Current voice id is {}".format(voice.id))
            engine.setProperty('voice', voice.id)
            engine.say('Drawing.')

    engine.say('Finish Drawing.')
    time.sleep(0.5)
    engine.say('Oh yeah')
    time.sleep(0.5)
    engine.say('Oh yeah')
    time.sleep(0.5)
    engine.say('Oh yeah')
    engine.runAndWait()


if __name__ == "__main__":
    play_sound()