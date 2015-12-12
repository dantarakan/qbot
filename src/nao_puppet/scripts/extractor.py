#!/usr/bin/env python

#
# Copyright (c) 2015 Miguel Sarabia
# Imperial College London
#
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#

#===============================================================================
# IMPORTS
#===============================================================================
import os
import fnmatch
import re
import csv
import rosbag
import datetime
import json
import argparse

#===============================================================================
# CONSTANTS
#===============================================================================
class Constants:

    exercises = [
        "puppet_exercise/exercise_left_arm", 
        "puppet_exercise/exercise_right_arm" ]

    songs = [
        "classical_music/carol_of_the_bells",
        "classical_music/mozart",
        "classical_music/beethoven",
        "classical_music/brahms",
        "classical_music/extra_carol1",
        "classical_music/extra_carol2",
        "classical_music/extra_carol3"]

    dances = [
        "taichi/dance",
        "alize-dance/dance" ]
        
    jokes = [
        "How do you know that carrots are good for your eyesight?",
        "What do snowmen have for breakfast?",
        "What do you have in December that you don't have in any other month?",
        "What does one ho plus two ho make?",
        "Three elderly ladies were at the doctor for a cognitive reasoning test.",
        "An elderly couple was driving across the country. While the woman was behind the wheel, the couple was pulled over by the highway patrol.",
        "A Horse goes into a bar and the bartender says \"Hey buddy, Why the Long Face?\"",
        "Where do you find a one legged dog?",
        "What is pink and fluffy?",
        "Two muffins are in the oven. One says to the other \"God it's hot in here\"" ]

    poetry = ["If you can fill the unforgiving minute. With sixty seconds worth"
        " of distance run. Yours is the Earth and everything that's in it. And,"
        " which is more, you'll be a Man, my son!"]

#===============================================================================
# HELPER FUNCTIONS
#===============================================================================
def extract_data_from_csv(csv_reader, data):
    #Initialise data if needed
    if not "participants" in data:
        data["participants"] = {}
    if not "refusals" in data:
        data["refusals"] = {}
        
    for row in csv_reader:
        name = row["Subject"].title()
        try:
            age = int( row["Age"] )
        except ValueError:
            age = None
            
        gender = "male" if row["Gender"].lower() == "m" else "female"
        refused = row["Refusal"].lower() == "yes"

        #Add refused to list of refusals        
        if refused:
            if name in data["refusals"]:
                raise RuntimeError("{} already processed!".format(name))
            
            data["refusals"][name] = {
                "name" : name,
                "age": age,
                "gender": gender }
            continue
        
        interactions = int(row["Number of Interactions"])
        has_dementia = "dementia" in row["Comments"].lower()
        eye_contact = row["Eye-contact"].lower() == "Full"
        verbal_interaction = row["Verbal interaction"].lower() == "Full"
        participant_exercised = row["Exercise Interaction"].lower() == "Full"
        
        if name in data["participants"]:
            raise RuntimeError("{} already processed!".format(name))
            
        #Save record        
        data["participants"][name] = {
            "name" : name,
            "age" : age,
            "gender" : gender,
            "interactions" : interactions,
            "has_dementia": has_dementia,
            "eye_contact": eye_contact,
            "verbal_interaction": verbal_interaction,
            "robot_exercised": False,
            "robot_sang": False,
            "robot_danced": False,
            "robot_joked": False,
            "robot_read_poetry": False,
            "participant_exercised": participant_exercised }
    
    return data

def extract_questionnaires_from_csv(csv_reader, data):
    if not "questionnaires" in data:
        data["questionnaires"] = []    
    for row in csv_reader:
        try:
            age = int( row["Age"] )
        except ValueError:
            age = None
        
        try:
            gender = "male" if row["Gender"].lower() == "male" else "female"
        except ValueError:
            gender = None
        
        enjoyed_robot = row[ "Have you enjoyed interaction with the robot "
            "during your time in the hospital" ].lower()
        would_use_robot_again = row[ "Would you want to "
            "use the robot again?" ].lower()
        useful_in_the_future = row[ "Do you think the robot will be a useful "
            "tool in patient care in the future?" ].lower()
        
        data["questionnaires"].append({
            "age" : age,
            "gender" : gender,
            "enjoyed_robot" : enjoyed_robot,
            "would_use_robot_again" : would_use_robot_again,
            "useful_in_the_future" : useful_in_the_future })
            
    return data
        
        
def get_bag_datetime(filename):

    match = re.search(
        "([0-9]{4})-([0-9]){2}-([0-9]{2})-([0-9]{2})-([0-9]{2})-([0-9]{2})",
        filename)

    dt = datetime.datetime(
         year = int(match.group(1)),
         month = int(match.group(2)),
         day = int(match.group(3)),
         hour = int(match.group(4)),
         minute = int(match.group(5)),
         second = int(match.group(6)))

    return dt

def get_bag_subject(filename):
    match = re.search("nao_puppet_[0-9\-]*_([a-zA-Z0-9\-\_]*).bag$", filename)
    if match is None:
        return None

    #Return name of subject (lower case and with spaces replacing underscores)
    return match.group(1).title().replace("_", " ")

def append_bag_data(bagfile, data):

    name = get_bag_subject(bagfile)
    if name not in data["participants"]:
        print("WARNING: {} not found in participants".format(name) )
        return data
        
    date = get_bag_datetime(bagfile)
    bag = BagExtractor(bagfile)
    
    entry = data["participants"][name]
    interaction = { "duration": bag.duration, "date" : str(date) }
    
    if "first_interaction" not in entry:
        entry["first_interaction"] = interaction
    else:
        entry["second_interaction"] = interaction
        
    if bag.robot_exercised:
        print("*** Check {} for exercises and update JSON".format(bagfile) )
        entry["robot_exercised"] = True
    if bag.robot_sang:
        entry["robot_sang"] = True
    if bag.robot_danced:
        entry["robot_danced"] = True
    if bag.robot_joked:
        entry["robot_joked"] = True
    if bag.robot_read_poetry:
        entry["robot_read_poetry"] = True
    
    return data  

def verify_data(extracted_data):
    for name, entry in extracted_data["participants"].items():
        if entry["interactions"] > 0 and "first_interaction" not in entry:
            raise RuntimeError(
                "Missing interaction file for {}".format( name ) )
        elif entry["interactions"] > 1 and "second_interaction" not in entry:
            raise RuntimeError(
                "Missing second interaction file for {}".format( name ) )
        elif entry["participant_exercised"] and not entry["robot_exercised"]:
            raise RuntimeError(
                "Conflicting exercise interaction data for {}".format(name) )
        if not entry["robot_exercised"]:
            entry["participant_exercised"] = None

    
    return extracted_data
                
#===============================================================================
# MAIN
#===============================================================================
class BagExtractor:
    def __init__(self, filename):
        self.bag = rosbag.Bag(filename)
        self.valid = True

        self.analyse()
        
        self.bag.close()
        self.valid = False
        

    def get_first_of(self, topic):
        if not self.valid:
            return

        for item in self.bag.read_messages(topics = [topic]):
            return item

        return None


    def get_last_of(self, topic):
        if not self.valid:
            return

        result = None
        for item in self.bag.read_messages(topics = [topic]):
            result = item

        return result
    
    def msg_was_sent(self, topic, msgs):
        if not self.valid:
            return 

        for item in self.bag.read_messages(topics = [topic]):
            if item[1].data in msgs:
                return True

        return False
        
    def analyse(self):
        if not self.valid:
            return
            
        start = self.get_first_of("/speech")
        end = self.get_last_of("/speech")
        
        if start is None or end is None:
            print("WARNING: Failed to get start/end of interaction")
            self.duration = None
        else:
            self.duration = (end[2] - start[2]).to_sec()
        
        #Check if nao offered to do any exercise        
        self.robot_exercised = self.msg_was_sent(
            "/behaviour",
            Constants.exercises )
        
        self.robot_sang = self.msg_was_sent(
            "/behaviour",
            Constants.songs )
        
        self.robot_danced = self.msg_was_sent(
            "/behaviour",
            Constants.dances )
        
        self.robot_joked = self.msg_was_sent(
            "/speech",
            Constants.jokes )
        
        self.robot_read_poetry = self.msg_was_sent(
            "/speech",
            Constants.poetry )
        


#===============================================================================
# MAIN
#===============================================================================
if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
        description = "Assemble a json script with all data."
        "Searches inside rosbags for all rosbags and reads data.csv and "
        "questionnaire.csv for extra informaton." )
    argparser.add_argument(
        "-o", "--output",
        help = "Name of output file (default: extracted_data.json)",
        default = "extracted_data.json")
    argparser.add_argument("--dir", help = "Change current directory to DIR")
    args = argparser.parse_args()

    if args.dir:
        os.chdir(args.dir)

    extracted_data = {}
    bags = []

    # Get all bags
    for root, dirnames, filenames in os.walk('rosbags'):
        for filename in fnmatch.filter(filenames, '*.bag'):
            bags.append(os.path.join(root, filename))

    with open("data.csv") as csvfile:
        print("Reading data.csv\n")
        reader = csv.DictReader(csvfile)
        extracted_data = extract_data_from_csv(reader, extracted_data)

    with open("questionnaire.csv") as csvfile:
        print("Reading questionnaires.csv\n")
        reader = csv.DictReader(csvfile)
        extracted_data = extract_questionnaires_from_csv(reader, extracted_data)    
  
    # Sort bags by date
    bags = sorted(bags, key = lambda bag: get_bag_datetime(bag))

    #Extract more info from rosbags
    for bagfile in bags:
        print("Analysing file: " + bagfile)
        extracted_data = append_bag_data(bagfile, extracted_data)
        
        print("")

    #Verify data
    extracted_data = verify_data( extracted_data )
    
    #Dump extracted_data as json file
    with open(args.output, "w") as jsonfile:
        json.dump(extracted_data, jsonfile, indent = 2, sort_keys=True)
