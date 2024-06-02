#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import pyttsx3
import time

class DescriptionNode:
    def __init__(self):
        rospy.init_node('description_node', anonymous=True)
        self.engine = self.initialize_tts_engine()
        self.described_statues = set()  # To keep track of described statues
        self.all_statues = {'King-Tut', 'Nefertiti', 'Chopin', 'Chinese Fu Dog'}  # Set of all statue names

        if not self.engine:
            rospy.logerr("Text-to-speech engine initialization failed.")
            return

        # Set the speech rate to be slower
        self.set_speech_rate(150)  # Adjust the rate value as needed (default rate is usually 200)

        self.finished_pub = rospy.Publisher('/description/finished', String, queue_size=10)
        self.done_pub = rospy.Publisher('/description/all_done', String, queue_size=10)
        rospy.Subscriber('/object_detection/class', String, self.description_callback)

    def initialize_tts_engine(self):
        try:
            engine = pyttsx3.init(driverName='espeak')
            return engine
        except Exception as e:
            rospy.logerr(f"Failed to initialize espeak driver: {e}")

        try:
            engine = pyttsx3.init(driverName='nsss')
            return engine
        except Exception as e:
            rospy.logerr(f"Failed to initialize nsss driver: {e}")

        try:
            engine = pyttsx3.init(driverName='sapi5')
            return engine
        except Exception as e:
            rospy.logerr(f"Failed to initialize sapi5 driver: {e}")

        rospy.logerr("All attempts to initialize pyttsx3 failed.")
        return None

    def set_speech_rate(self, rate):
        self.engine.setProperty('rate', rate)

    def say_and_sleep(self, text, sleep_duration):
        self.engine.say(text)
        self.engine.runAndWait()
        time.sleep(sleep_duration)

    def description_callback(self, statues_name):
        if statues_name.data in self.described_statues:
            rospy.loginfo(f"Skipping already described statue: {statues_name.data}")
            return

        rospy.loginfo(f"Describing statue: {statues_name.data}")

        # King Tutankhamun description
        if statues_name.data == 'King-Tut':
            self.say_and_sleep("Welcome, ladies and gentlemen, to the captivating world of Tutankhamun.", 4)
            # self.say_and_sleep("The legendary 'Boy King' of ancient Egypt.", 2)
            # self.say_and_sleep("Tutankhamun, born circa 1341 BC.", 2)
            # self.say_and_sleep("He ascended the throne at the tender age of nine and ruled for a short but intriguing reign.", 4)
            # self.say_and_sleep("His tomb, discovered nearly intact in the Valley of the Kings in 1922 by Howard Carter.", 4)
            # self.say_and_sleep("Unveiled treasures that have captured the imagination of the world ever since.", 2)
            # self.say_and_sleep("Tutankhamun remains a symbol of Egypt's rich history and mysteries waiting to be unraveled.", 0)

        # Queen Nefertiti description
        elif statues_name.data == 'Nefertiti':
            self.say_and_sleep("To the enchanting tale of Nefertiti, the illustrious Queen of Egypt.", 4)
            # self.say_and_sleep("Revered for her unparalleled beauty and grace, Nefertiti graced the courts of ancient Egypt.", 2)
            # self.say_and_sleep("Alongside her husband, Pharaoh Akhenaten, during the intriguing period of the Amarna era.", 4)
            # self.say_and_sleep("Her iconic bust, discovered in 1912, captivates with its timeless elegance, symbolizing not just her physical allure,", 4)
            # self.say_and_sleep("But also her powerful influence as a consort and perhaps even a co-regent.", 2)
            # self.say_and_sleep("Join me as we journey through the life and legacy of this enigmatic queen,", 2)
            # self.say_and_sleep("Whose name means 'the beautiful one has come.'", 0)

        # Frederic Chopin description
        elif statues_name.data == 'Chopin':
            self.say_and_sleep("To the musical world of Frédéric Chopin, the renowned Polish composer and virtuoso pianist.", 4)
            # self.say_and_sleep("Born in 1810, Chopin's compositions, such as his haunting nocturnes and passionate ballades, continue to captivate audiences worldwide.", 2)
            # self.say_and_sleep("His innovative style and emotive melodies earned him the title 'Poet of the Piano.'", 4)
            # self.say_and_sleep("Join me as we explore the life and legacy of this musical genius,", 2)
            # self.say_and_sleep("Whose works resonate with timeless beauty and profound emotion,", 2)
            # self.say_and_sleep("Enriching the soul of music lovers for generations to come.", 4)

        # Chinese Fu Dog description
        elif statues_name.data == 'Chinese Fu Dog':
            self.say_and_sleep("Step into our museum and encounter the regal presence of a Chinese Fu Dog statue.", 4)
            # self.say_and_sleep("With its guardian stance and intricate detailing,", 2)
            # self.say_and_sleep("It embodies ancient myths and cultural significance.", 2)
            # self.say_and_sleep("Crafted with precision, it stands as a symbol of protection and prosperity,", 4)
            # self.say_and_sleep("Inviting you to explore the richness of Chinese tradition and heritage.", 0)

        # Add the described statue to the set
        self.described_statues.add(statues_name.data)

        # Publish a message indicating the description has finished
        self.finished_pub.publish(statues_name.data)
        rospy.loginfo(f"Finished description for {statues_name.data}")

        # Check if all statues are described
        if self.described_statues == self.all_statues:
            rospy.loginfo("All statues have been described.")
            self.done_pub.publish("All statues have been described.")
            rospy.signal_shutdown("All statues have been described.")  # Optionally shut down the node

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        description_node = DescriptionNode()
        description_node.run()
    except rospy.ROSInterruptException:
        pass
