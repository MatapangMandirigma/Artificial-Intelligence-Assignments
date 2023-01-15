import sys
import copy

# Class for saving the variables of the Resolution
class Resolution:
    clauseList = None
    testClause = None

    # parameterized constructor
    def __init__(self, clauseList, testClause, clauseCount):
        self.clauseList = clauseList
        self.testClause = testClause
        self.clauseCount = clauseCount
        # Run initial functions for resolutions
        self.negateTestClause()
        self.removeDuplicates()
        self.resByRef()

    # Remove duplicates from the clause list
    def removeDuplicates(self):
        # Set the count fot the duplicate removal
        count = self.clauseCount

        # For loop for every clause in the original clauseList
        for clause in range(count):
            
            # Run the helper function
            self.removeDuplicatesHelper(clause+1)
            
        # Set count to 1
        # Initialize updateClauseList dictionary
        count = 1
        updatedClauseList = {}

        # For loop to make an updatedClauseList
        for clause in self.clauseList:
            updatedClauseList[count] = self.clauseList[clause]
            count+=1

        # Update the clauseList with updatedClauseList
        self.clauseList = updatedClauseList

        return

    # Helper function to remove duplicates
    def removeDuplicatesHelper(self, clause):
        # For loop that checks for compClauses that could be a duplicate of the clause
        for compClause in self.clauseList:
            if(clause != compClause):

                # Find a duplicate, if there is a duplicate then remove the duplicate
                duplicate = self.compareClauses(clause, compClause)
                if(duplicate):
                    self.clauseList.pop(clause)
                    self.clauseCount-=1
                    break
        return

    # Compare if clauses are the same function
    def compareClauses(self, clause, compClause, newClause=None):
        # If the compare is from the resolutaion by refutation
        if(newClause is not None):

            # If the clauses have the same number of literals
            if(len(self.clauseList[clause].literals) == len(newClause.literals)):

                # Nested for loop to check a literal with all compLiterals
                for literal in self.clauseList[clause].literals:
                    for compLiteral in newClause.literals:

                        # If statement to check if the literal is equal to the compLiteral
                        literalFound = False
                        if((literal.literal == compLiteral.literal) and (literal.negation == compLiteral.negation)):
                            literalFound = True
                            break
                    
                    # If the literal was not found the return false
                    if(not literalFound):
                        return False
                # Since literal was found return true
                return True

            # Otherwise return false
            else:
                return False

        # Comments are the same from if statement
        else:
            if(len(self.clauseList[clause].literals) == len(self.clauseList[compClause].literals)):
                for literal in self.clauseList[clause].literals:
                    for compLiteral in self.clauseList[compClause].literals:
                        literalFound = False
                        if((literal.literal == compLiteral.literal) and (literal.negation == compLiteral.negation)):
                            literalFound = True
                            break
                    if(not literalFound):
                        return False
                return True
            else:
                return False

    # Negate the clause for resolution
    def negateTestClause(self):
        # Create and copy of the testClause
        testClause = copy.deepcopy(self.testClause)

        # For loop for the literals in the testClause
        for literal in testClause.literals:
            newClause = []
            self.clauseCount+=1

            # Change the negation of the literal to the opposite
            if(literal.negation == True):
                literal.negation = False
            else:
                literal.negation = True

            # Add literal to the newClause
            newClause.append(literal)

            # Add the newClause to clauseList
            self.clauseList[self.clauseCount] = Clause(newClause, None)

        return
    
    # Resolution by refutation function
    def resByRef(self):
        # Set the count to 1 for looping
        # Initialize the contradiction boolean
        count = 1
        contradictionFound = False

        # While count is less than the size of the clauseList
        while (count <= len(self.clauseList)):

            # For loop to run through the claues in clauseList
            for compClause in range(len(self.clauseList)):

                # If the clauses are previous clauses then compare clauses and look for contradiction
                if(compClause+1 < count):
                    contradictionFound = self.resByRefCompare(count, compClause+1)
                    if(contradictionFound):
                        break
            # If contradiction is found then break while loop and return
            if(contradictionFound):
                break
            count+=1 #

        # If contradiction if found then print valid solution
        # Otherwise print fail solution
        if(contradictionFound):
            self.solutionPrint(True)
        else:
            self.solutionPrint(False)
            
        return
    
    # Resolution by refutation clause compare function
    def resByRefCompare(self, clause, compClause):
        # Initialize the clauses that can be negated
        clauseCancelCount = 0

        # Nesting for loop to run through the literals of the clause and compared clause
        for literal in self.clauseList[clause].literals:
            for compLiteral in self.clauseList[compClause].literals:

                # If the literals would negate each other, set the negatedLiteral, set the clauses that created
                # the new clause and increment the negated clause count
                if((literal.literal == compLiteral.literal) and (literal.negation != compLiteral.negation)):
                        negatedLiteral = literal.literal
                        creationByClauses = [clause, compClause]
                        clauseCancelCount += 1
        
        # If the amount of negated literals is one
        # Otherwise continue on
        if(clauseCancelCount == 1):
            # Initialize the newLiterals list
            newLiterals = []

            # Add the literals that aren't negated to the newLiterals list
            for literal in self.clauseList[clause].literals:
                if(literal.literal != negatedLiteral):
                    newLiterals.append(literal)
            
            # Add the compLiterals that aren't negated and aren't already in the list
            for compLiteral in self.clauseList[compClause].literals:
                if(compLiteral.literal != negatedLiteral):
                    sameLiteral = False
                    for literal in newLiterals:
                        if(compLiteral.literal == literal.literal):
                            sameLiteral = True
                    if(not sameLiteral):
                        newLiterals.append(compLiteral)
            
            # If the list is empty
            # Otherwise add it to the clauseList
            if(len(newLiterals) == 0):
                # Set the clause as a contradiction
                newLiterals.append(Literal('Contradiction', False))
                newClause = Clause(newLiterals, creationByClauses)

                # Add the newClause to the clauseList and increment the clauseCount
                self.clauseCount+=1
                self.clauseList[self.clauseCount] = Clause(newLiterals, creationByClauses)

                return True
            else:
                # Set the newClause with newLiterals list and creationByClauses
                newClause = Clause(newLiterals, creationByClauses)

                # For loop to check for duplicates
                for clause in self.clauseList:
                    duplicate = self.compareClauses(clause, clause, newClause)
                    # If there is a duplicate then just break the for loop
                    if(duplicate):
                        break

                # If the clause isn't a duplicate then 
                # add the newClause to the clauseList and increment the clauseCount
                if(not duplicate):
                    self.clauseCount+=1
                    self.clauseList[self.clauseCount] = Clause(newLiterals, creationByClauses)

        return False

    # Print out of the solution that was found
    def solutionPrint(self, answer):
        count = 1
        for clause in self.clauseList:
            print(count, '. ', end='', sep='')
            literalCount = 1
            for literal in self.clauseList[clause].literals:
                if(literal.negation == True):
                    print('~', literal.literal, end='', sep='')
                else:
                    print(literal.literal, end='', sep='')
                if(literalCount != len(self.clauseList[clause].literals)):
                    print(' ', end='', sep='')
                literalCount+=1
            if(self.clauseList[clause].creationByClauses is not None):
                print(' {', self.clauseList[clause].creationByClauses[0], ',', self.clauseList[clause].creationByClauses[1], '}', sep='')
            else:
                print(' {}')
            count+=1
        
        if(answer):
            print('Valid')
        else:
            print('Fail')

    # Test print for debugging
    def testPrint(self):
        for clause in self.clauseList:
            count = 1
            for literal in self.clauseList[clause].literals:
                if(literal.negation == True):
                    print('~', literal.literal, end='', sep='')
                else:
                    print(literal.literal, end='', sep='')
                if(count != len(self.clauseList[clause].literals)):
                    print(' ', end='', sep='')
                count+=1
            print()
        count = 1
        for literal in self.testClause.literals:
            if(literal.negation == True):
                print('~', literal.literal, end='', sep='')
            else:
                print(literal.literal, end='', sep='')
            if(count != len(self.testClause.literals)):
                print(' ', end='', sep='')
            count+=1
        print()

# Class for saving the variables of the Clause
class Clause:
    literals: None
    creationByClauses: None

    # parameterized constructor
    def __init__(self, literals, creationByClauses):
        self.literals = literals
        self.creationByClauses = creationByClauses

# Class for saving the variables of the Literal  
class Literal:
    literal: None
    negation: None

    # parameterized constructor
    def __init__(self, literal, negation):
        self.literal = literal
        self.negation = negation

# Main function
def main():

    # Save clauses in a clauseList
    clauseList = {}
    clauseCount = 0 # Save clause clount for entry into the dictionary

    # Save the file given from the command arguments
    with open(sys.argv[1]) as clauseInFile:
        # For loop to go through eah clause line
        for clauseLine in clauseInFile:
            clauseLine = clauseLine.strip()

            # Increment the clauseCount and initialize newClause list
            clauseCount += 1
            newClause = []

            # For to loop to get the literals in the clauseLine
            for literal in clauseLine[0:].split(' '):

                # If literal has ~ then set literal with negation true
                # Otherwise set literal with negation false
                if('~' in literal):
                    newLiteral = Literal(literal[1], True)
                else:
                    newLiteral = Literal(literal[0], False)

                # Add literal to clause
                newClause.append(newLiteral)

            # Add clause to clauseList dictionary
            clauseList[clauseCount] = Clause(newClause, None)

        # Set testClause as last clause read in for clauseList
        testClause = clauseList.pop(clauseCount)
    
        # Run the resolution function
        Resolution(clauseList, testClause, clauseCount-1)

    return

# Call the main function
if __name__ == "__main__":
    main()