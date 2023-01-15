# Code done by Brandon Angeles and Zaid Habibi

import sys
import copy

# Class for saving the variables of the CSP
class Var:
    assignedValue = None
    Label = None
    Domain = None

    # parameterized constructor
    def __init__(self, assignedValue, Label, Domain):
        self.assignedValue = assignedValue
        self.Label = Label
        self.Domain = Domain

# Class for the CSP
# Includes backtracking, printing, etc.
class CSP:
    counter = None
    assignmentList = None
    variableList = None
    constraintList = None
    fc = None

    # parameterized constructor
    def __init__(self, counter, assignmentList, variableList, constraintList, fc):
        self.counter = counter
        self.assignmentList = assignmentList
        self.variableList = variableList
        self.constraintList = constraintList
        self.fc = fc

    # backtracking function for CSP
    def backtrackingFunction(self):
        # Check at the beginning if all variables have been assigned a value
        allAssigned = True
        for assignedVariable in self.variableList.values():
            if assignedVariable.assignedValue == None:
                allAssigned = False

        if allAssigned:
            return self.assignmentList
        
        # Use MCV to select an unassigned variable
        selectedVariable = self.mostConstrainedVariable()

        # Use orderedDomain to help with LCV later on
        orderedDomain = self.orderDomain(selectedVariable)

        # Now iterate through the domain and see which value works with LCV and the constraints with it
        for values in orderedDomain: 
            for value in values:
                constraintsPassed = True

                # checking each constraint
                for constraint in self.constraintList:

                    # If the variable is on the left side of the constriant check if the asssigned value would work
                    if(constraint[0] == self.variableList[selectedVariable].Label and 
                    self.variableList[constraint[2]].assignedValue is not None):
                        constraintsPassed = self.constraintCheck(0, value, constraint)

                    # If the variable is on the right side of the constriant check if the asssigned value would work
                    elif(constraint[2] == self.variableList[selectedVariable].Label and 
                    self.variableList[constraint[0]].assignedValue is not None):
                        constraintsPassed = self.constraintCheck(2, value, constraint)

                    # If at any time the constraint would not work then print the failure
                    if not constraintsPassed:
                        self.failurePrint(selectedVariable, value)
                        break
                # If the value passes all the constraints then assign it to the variable
                if constraintsPassed:
                    self.variableList[selectedVariable].assignedValue = value
                    self.assignmentList[selectedVariable] = value
                    
                    # Save for resetting
                    originalVariableList = copy.deepcopy(self.variableList)

                    # If forward checking is true then go through this loop
                    if self.fc:
                        # Run the algorithm and save the variable list
                        tempVariableList = self.forwardCheckingAlgo(selectedVariable)
                        # Now check if any of the domains are empty and if so print failure
                        for variable in tempVariableList.values():
                            if len(variable.Domain) is 0:
                                self.failurePrint(selectedVariable, value)
                                continue
                        # Set the varibleList equal to the tempVariableList for forward checking 
                        self.variableList = copy.deepcopy(tempVariableList)

                    # Rerun the recursion for the backtracking function
                    result = self.backtrackingFunction()

                    # If the result is not false then return this result to print solution
                    if result is not False:
                        return result
                    # Otherwise let's reset everything back to where it was before the recursion call
                    self.variableList = copy.deepcopy(originalVariableList)
                    self.variableList[selectedVariable].assignedValue = None
                    self.assignmentList.pop(selectedVariable)
        
        # Otherwise return false
        return False

    # used by backtracking to check for the MCV
    def mostConstrainedVariable(self):
        currentVariable = None

        # Run through the variables of the variableList
        for variable in self.variableList.keys():
            # If variable is unassigned
            if(self.variableList[variable].assignedValue is None):
                # If not variable is being checked right now add the variables being checked to the currentVariable 
                if(currentVariable is None):
                    currentVariable = variable

                # If the domain of the new variable is less than the currentVariable, set the currentVariable to variable
                elif(len(self.variableList[currentVariable].Domain) > len(self.variableList[variable].Domain)):
                    currentVariable = variable

                # Otherwise if the domains are the same between currentVariable and variable, run through the most constraining
                # variable tiebreaker
                elif(len(self.variableList[currentVariable].Domain) == len(self.variableList[variable].Domain)):
                    currentVariableConstraints = 0
                    variableConstraints = 0

                    # Run through the rest of the constraints
                    for constraint in self.constraintList:
                        # Now check if the currentVariable is a part of this constraint and if the other variable is unassigned. 
                        # If so increment the currentVariableConstraints
                        if((self.variableList[currentVariable].Label == constraint[0] and self.variableList[constraint[2]].assignedValue is None) 
                        or (self.variableList[currentVariable].Label == constraint[2] and self.variableList[constraint[0]].assignedValue is None)):
                            currentVariableConstraints += 1
                        # Now check if the variable is a part of this constraint and if the other variable is unassigned. 
                        # If so increment the variableConstraints
                        if((self.variableList[variable].Label == constraint[0] and self.variableList[constraint[2]].assignedValue is None) 
                        or (self.variableList[variable].Label == constraint[2] and self.variableList[constraint[0]].assignedValue is None)):
                            variableConstraints += 1
                    # If the variable has more constraints than the currentVariable, set the currentVariable to variable
                    if(currentVariableConstraints < variableConstraints):
                        currentVariable = variable

        # Return the currentVariable
        return currentVariable

    # used for ordering which domain is the most constrained and for LCV later on
    def orderDomain(self, variable):
        # Dictionary to save the values of the constraints for each value
        variableConstrainingValues = {}

        # Run through the values in the selected variable's domain
        for value in self.variableList[variable].Domain:
            constraintsValue = 0

            # Run through the constraints
            for constraint in self.constraintList:
                # If the variable is in the 0 position of the constraint and then other doesn't have a value
                if(self.variableList[variable].Label == constraint[0] and 
                self.variableList[constraint[2]].assignedValue is None):
                    # Now run thorugh the other variables domain and if the constraint doesn't pass with the new assigned value
                    # increment the constraintsValue
                    for otherValue in self.variableList[constraint[2]].Domain:
                        if(not self.constraintCheck(0, value, constraint, otherValue)):
                            constraintsValue += 1

                # If the variable is in the 2 position of the constraint and then other doesn't have a value
                elif(self.variableList[variable].Label == constraint[2] and 
                self.variableList[constraint[0]].assignedValue is None):
                    # Now run thorugh the other variables domain and if the constraint doesn't pass with the new assigned value
                    # increment the constraintsValue
                    for otherValue in self.variableList[constraint[0]].Domain:
                        if(not self.constraintCheck(2, value, constraint, otherValue)):
                            constraintsValue += 1
            
            # If the value of the constraintsValue is already in the dictionary
            # then add it to the constraintValue's list 
            if(constraintsValue in variableConstrainingValues):
                variableConstrainingValues[constraintsValue].append(value)
            # Otherwise add a new value to the dictionary with the constraintsValue as a key
            else:
                variableConstrainingValues[constraintsValue] = [value]
        
        # domainInOrder list
        domainInOrder = []
        # Now add the values from the dictionary sorted from least to most constraints 
        for domain in sorted(variableConstrainingValues.keys()):
            domainInOrder.append(variableConstrainingValues[domain])

        return domainInOrder

    # forward checking for the CSP
    def forwardCheckingAlgo(self, selectedVariable):
        # Save the assigned value of the selected variable
        assignedValue = self.variableList[selectedVariable].assignedValue

        # Save the variableList in CSP to tempVariableList to change
        tempVariableList = copy.deepcopy(self.variableList)

        # Run through all the constraints to change the domains
        for constraint in self.constraintList:
            # If the variable is in the 0 position of the constraint and then other doesn't have a value
            if(tempVariableList[selectedVariable].Label == constraint[0] and 
            tempVariableList[constraint[2]].assignedValue is None):
                # Now run thorugh the other variables domain and if the constraint doesn't pass with the new assigned value
                # remove it from the variable's domain
                for value in self.variableList[constraint[2]].Domain:
                    if(not self.constraintCheck(0, assignedValue, constraint, value)):
                        tempVariableList[constraint[2]].Domain.remove(value)
            # If the variable is in the 2 position of the constraint and then other doesn't have a value
            elif(tempVariableList[selectedVariable].Label == constraint[2] and 
            tempVariableList[constraint[0]].assignedValue is None):
                # Now run thorugh the other variables domain and if the constraint doesn't pass with the new assigned value
                # remove it from the variable's domain
                for value in self.variableList[constraint[0]].Domain:
                    if(not self.constraintCheck(2, assignedValue, constraint, value)):
                        tempVariableList[constraint[0]].Domain.remove(value)

        # Return the tempVariableList with the new domains
        return tempVariableList
    
    # used to check if the values being tested follow the constraints
    def constraintCheck(self, position, value, constraint, domainValue=None):
        compareValue = 0
        # Used for knowing how to compare the values entered
        if(position == 0):
            if(domainValue is not None):
                compareValue = domainValue
            else:
                compareValue = self.variableList[constraint[2]].assignedValue

            # Check depending on what constraint if it is true or not with the values given
            if constraint[1] == '>':
                if(value > compareValue):
                    return True
            elif constraint[1] == '<':
                if(value < compareValue):
                    return True
            elif constraint[1] == '=':
                if(value == compareValue):
                    return True
            elif constraint[1] == '!':
                if(value != compareValue):
                    return True
            else:
                print("Invalid constraint")
                SystemExit
        
        # Used for knowing how to compare the values entered
        elif(position == 2):
            if(domainValue is not None):
                compareValue = domainValue
            else:
                compareValue = self.variableList[constraint[0]].assignedValue

            # Check depending on what constraint if it is true or not with the values given
            if constraint[1] == '>':
                if(compareValue > value):
                    return True
            elif constraint[1] == '<':
                if(compareValue < value):
                    return True
            elif constraint[1] == '=':
                if(compareValue == value):
                    return True
            elif constraint[1] == '!':
                if(compareValue != value):
                    return True
            else:
                print("Invalid constraint")
                SystemExit

        # Otherwise return false
        return False

    # print out the failure of the assigned variables
    def failurePrint(self, selectedVariable, value):
        c = 0
        self.counter += 1
        print(self.counter, ". ", end="", sep="")
        for variable in self.assignmentList.keys():
            if c is len(self.assignmentList.keys()) - 1:
                if(not self.fc):
                    print(variable, "=", self.assignmentList[variable], ", ", end="", sep="")
                print(self.variableList[selectedVariable].Label, "=", value, " failure", sep="")
            else:
                print(variable, "=", self.assignmentList[variable], ", ", end="", sep="")
                c += 1
        if self.counter >= 30:
            SystemExit
        return

    # used for debugging
    def testPrint(self):
        for varLine in self.variableList:
            out = self.variableList[varLine].Label + ":"
            for i in self.variableList[varLine].Domain:
                out += str(i)
            print(out)
        for consLine in self.constraintList:
            print(consLine)
        print(self.fc)

# main function
def main():

    # Save variables in a varList
    varList = {}
    with open(sys.argv[1]) as varInFile:
        for varLine in varInFile:
            varLine = varLine.strip()

            newLabel = varLine[0]
            newDomain = []
            for value in varLine[3:].split(' '):
                newDomain.append(int(value))
            varList[newLabel] = Var(None, newLabel, newDomain)

    # Save constraints in consList
    consList = []
    with open(sys.argv[2]) as consInFile:
        for consLine in consInFile:
            consLine = consLine.strip()
            consLine = consLine.replace(' ', '')
            consList.append(consLine)

    # Save whether forward checking or not     
    if sys.argv[3] == 'none':
        fc = False
    elif sys.argv[3] == 'fc':
        fc = True
    else:
        print('Invalid input, please put none or fc')
        return

    # Save as a CSP
    assignedValues = {}
    csp = CSP(0, assignedValues, varList, consList, fc)

    # Run the backtrackingFunction
    result = csp.backtrackingFunction()

    # If it didn't return false then print out the solution
    if(result is not False):
        c = 0
        csp.counter += 1
        print(csp.counter, ". ", end="", sep="")
        for variable in result.keys():
            if c is len(result.keys()) - 1:
                print(variable, "=", result[variable], " solution", sep="")
            else:
                print(variable, "=", result[variable], ", ", end="", sep="")
            c += 1

    return

# To call main in the beginning
if __name__ == "__main__":
    main()
