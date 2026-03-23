import json

#create a test class
def testObject(name,age, number):
    return  {
        f"{name}": {
            "age": age,
            "number": number
        }
    }
test = testObject("test", 100, 1234567890)
test2 = testObject("test2", 200, 1234567890)

data = {}

with open("TestFiles/test.json", "r") as jsonFile:
    data = json.load(jsonFile)
    data.update(test)
    data.update(test2)

with open("TestFiles/test.json", "w") as jsonFile:
    json.dump(data, jsonFile, indent=4)