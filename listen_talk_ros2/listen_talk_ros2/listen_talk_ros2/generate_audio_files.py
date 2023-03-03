# This files generates .mp3 files that will be used offline. You need to be connected to the
# internnet to run this and generate the audio files.
#
# Cannot work offline needs to be connected to the internet to convert text to speach, but
# Can load prewritten .mp3 files when not connected to the internet just use the command below:
# os.system('mpg123 talk.mp3')
#
# To see all available languages run this command in a terminal:
# gtts-cli --all

import os
from gtts import gTTS

myText = 'Hi my name is Willie and I am here to help.'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False)
myOutput.save('../config/willie_greet.mp3')
# Can load prewritten .mp3 files when not connected to the internet
os.system('mpg123 ../config/willie_greet.mp3')
# os.system('mpg123 <Path to file>')

myText = 'Allan! Allan! Allan! No it is Steve! Oh! Steve! Steve! Steve!'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False) # language - en, fr, es, it, de, in, uk
myOutput.save('../config/allan.mp3')

myText = 'Stairs in front of you'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False)
myOutput.save('../config/stairs_in_front.mp3')

myText = 'Stairs on the left'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False)
myOutput.save('../config/stairs_left.mp3')

myText = 'Stairs on the right'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False)
myOutput.save('../config/stairs_right.mp3')

myText = 'Whoof whoof whoof whoof'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False)
myOutput.save('../config/talk.mp3')

myText = 'I did not understand you. Please try again.'
myOutput = gTTS(text=myText, lang='en', slow=False, lang_check=False)
myOutput.save('../config/not_understanding.mp3')

myText = 'Thank you, thank you, thank you very much!'
myOutput = gTTS(text=myText, lang='en', slow=True, lang_check=False)
myOutput.save('../config/thank_you.mp3')
os.system('mpg123 ../config/thank_you.mp3')

myText = 'Ek will net baie dankie se vir Marno wat my gemaak het en ek hoop ek kan hom trots maak deur baie mense te help!'
myOutput = gTTS(text=myText, lang='af', slow=False, lang_check=False)
myOutput.save('../config/easter_egg.mp3')
