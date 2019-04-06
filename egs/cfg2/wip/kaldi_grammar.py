#! /usr/bin/env python
import os
from grammar_parser.cfgparser import CFGParser


class KaldiGrammar:
    """
    Class KaldiGrammar uses as input a grammar file with extension '.fcfg' and has two functions:
    get_rule_element: extracts the defined grammar rules
    get_words: extracts the unique words and creates 'corpus.txt' which is used to build 'G.fst'
    """
    def __init__(self, grammar_file_string, target):

        if os.path.exists(grammar_file_string):
            self.parser = CFGParser.fromfile(grammar_file_string)
            self.grammar_file = grammar_file_string
        else:
            self.parser = CFGParser.fromstring(grammar_file_string)
            self.grammar_string = grammar_file_string

        self.target = target

    # ----------------------------------------------------------------------------------------------------

    def get_rule_element(self, target, depth=0):
        """
        Extracts the grammar rules, defined within the grammar file

        :param depth: level of depth of the grammar rule
        :return RulesList: grammar tree
        """
        RulesList = {}
        # print(depth)
        if target not in self.parser.rules:
            raise Exception("Target {} not in parser rules".format(target))

        # If already present in RULES, return it
        if target in RulesList:
            return RulesList[target]

        # Get the rule
        rule = self.parser.rules[target]

        # Iterate over all options
        option_alternative_list = []
        for opt in rule.options:

            # Iterate over all conjunctions
            conjunctions_list = []
            for conj in opt.conjuncts:
                # If the conjunction is already present
                if conj.name in RulesList:
                    conjunctions_list.append(RulesList[conj.name])
                    continue

                # If variable: go one level deeper
                if conj.is_variable:
                    result = self.get_rule_element(conj.name, depth + 1)
                    if result:
                        conjunctions_list.append(result)
                else:
                    # Add a new literal to the list
                    RulesList[conj.name] = conj.name
                    conjunctions_list.append(RulesList[conj.name])

            if len(conjunctions_list) == 1:
                option_alternative_list.append(conjunctions_list[0])
            else:
                option_alternative_list.append(tuple(conjunctions_list))

        if len(option_alternative_list) == 1:
            RulesList[target] = option_alternative_list[0]
        else:
            RulesList[target] = option_alternative_list

        return RulesList[target]

    # ----------------------------------------------------------------------------------------------------

    def get_words(self):
        """
        Extracts list with all the unique words, used within the grammar and
        create file 'corpus.txt' which is used to build 'G.fst'
        """

        # Extract rules from the grammar file
        rules = self.parser.rules

        # Extract words
        words = set()

        for key, value in rules.iteritems():
            # Get the list of options for the rule value
            options = value.options
            for option in options:
                # Get the list of conjuncts for option 'option'
                conjuncts = option.conjuncts
                for conjunct in conjuncts:
                    # If conjunct is not a variable put its value in the set of words
                    if not conjunct.is_variable:
                        words.add(conjunct.name)

        words = [word.upper() + "\n" for word in list(words)]
        # print(words)

        # Create corpus.txt file and save the words list
        with open("corpus.txt", "w") as f:
            for word in words:
                f.write(word)

    # ----------------------------------------------------------------------------------------------------

    # TODO:
    # expand the full tree, not only the first words
    # replace raw_input with the speech recognition output
    # add an option to skip a word if it is not a match and to check the next word

    def autocomplete(self):

        recognised_sentence = []

        recognition = raw_input("Recognised word: ")
        type(recognition)

        # create a filtered list, based on the recognised first word
        initial_list, recognised = self.check_word(recognition)
        if not recognised:
            print('Not a match')
        else:
            # remove the first word from each line
            first_word = [line.pop(0) for line in initial_list]
            recognised_sentence.append(first_word[0])
            sentence_list = initial_list

            print('Initial filtered list: \n')
            self.print_nicely(sentence_list)

            while len(sentence_list[0]) > 0:
                next_recognition = raw_input("Next recognised word: ")
                type(next_recognition)

                # create a filtered list, based on the next recognised word
                new_initial_list, recognised = self.check_word(next_recognition, sentence_list)
                if not recognised:
                    print('Not a match')
                    break
                else:
                    # remove the first word from each line
                    next_word = [line.pop(0) for line in new_initial_list]
                    recognised_sentence.append(next_word[0])
                    sentence_list = new_initial_list

                    print('New filtered list: \n')
                    self.print_nicely(sentence_list)

        print('Recognised sentence: \n' + str(recognised_sentence))
        return recognised_sentence

    # ----------------------------------------------------------------------------------------------------

    def check_word(self, recognition='', initial_list=[]):
        """
        Checks if the recognised word is matching with the first element in the expanded sentences
        As output it keeps a list of only the sentences, starting with the recognised word.

        :param recognition: the recognised word
        :param initial_list: bla
        :return filtered_list: sentence list, filtered by its first word
        """

        recognised = False

        if len(initial_list) == 0:
            initial_list = self.expand_tree()

            filtered_list = []
            for sentence in initial_list:
                line = [item.name for item in sentence]
                if line[0] == recognition:
                    filtered_list.append(line)
                    continue

        else:
            filtered_list = []
            for sentence in initial_list:
                line = [item for item in sentence]
                if line[0] == recognition:
                    filtered_list.append(line)
                    continue

        if len(filtered_list) > 0:
            recognised = True

        print('Filtered list: \n')
        print(recognised)
        self.print_nicely(filtered_list)
        return filtered_list, recognised

    # ----------------------------------------------------------------------------------------------------

    def expand_tree(self):
        """
        Expands the grammar tree based on the first word in the rule.
        Used for validation of the first recognised word. 

        :return sentence_list: list of all possible sentences
        """

        # Extract rules from the grammar file
        rules = self.parser.rules

        sentence_list = [rules['T'].options[0].conjuncts[:]]

        while sentence_list:
            not_expanded = False
            for item in sentence_list:
                if item[0].is_variable:
                    not_expanded = True
                    break
            if not not_expanded:
                break

            expanded_list = []
            for item in sentence_list:
                if not item:
                    continue
                if not item[0].is_variable:
                    expanded_list.append(item)
                    continue

                for opt in rules[item[0].name].options:
                    d = opt.conjuncts + item[1:]
                    expanded_list.append(d)

            sentence_list = expanded_list
        return sentence_list

    # ----------------------------------------------------------------------------------------------------

    def print_nicely(self, sentence_list):
        """
        Prints cleanly the output of the tree traversal functions
        
        :param sentence_list: list of possible completions
        """
        for sentence in sentence_list:
            line = [item for item in sentence]
            print(" ".join(line))
        print('')

    # ----------------------------------------------------------------------------------------------------


class SentenceNode:
    """
    A node in a sentence.
    :ivar edges: Edges to the next node.
    :ivar done: Reached the end of the sentence.
    """
    def __init__(self):
        self.edges = []
        self.done = False

    # ----------------------------------------------------------------------------------------------------


class SentenceEdge:
    """
    An edge in a sentence.
    :ivar word: The word to be understood.
    :ivar node: Node for the remainder of the sentence.
    """
    def __init__(self, word):
        self.word = word
        self.node = SentenceNode()

    # ----------------------------------------------------------------------------------------------------


def expand_tree(rules):
    """
    Expands the grammar tree based on the first word in the rule.
    Used for validation of the first recognised word.

    :param rules: Extracted rules from the grammar file.
    :return: The root of the expanded tree.
    :rtype: SentenceNode
    """

    root_node = SentenceNode()

    sentence_list = [opt.conjuncts[:] for opt in rules['T'].options]

    work_list = [(root_node, sentence_list)]
    while work_list:
        node, unexpanded_list = work_list.pop()
        expanded = expand_sentences(unexpanded_list, rules)
        prefix_dict = {}
        for item in expanded:
            successors = prefix_dict.get(item[0].name)
            if successors:
                successors.append(item[1:])
            else:
                prefix_dict[item[0].name] = [item[1:]]
        for word, unexpanded in prefix_dict.items():
            edge = SentenceEdge(word)
            node.edges.append(edge)

            if any(unexpanded):
                unexpanded = [item for item in unexpanded if item]
                edge.node.done = True

            if unexpanded:
                work_list.append((edge.node, unexpanded))

    return root_node

    # ----------------------------------------------------------------------------------------------------


def expand_sentences(sentence_list, rules):
    """
    Expands the grammar rules until elimination of all variables at the first position

    :param sentence_list: List of grammar rules
    :param rules: Rules of the grammar
    :return: Expanded list
    """

    while sentence_list:
        not_expanded = False
        for item in sentence_list:
            if item[0].is_variable:
                not_expanded = True
                break
        if not not_expanded:
            break

        expanded_list = []
        for item in sentence_list:
            if not item:
                continue
            if not item[0].is_variable:
                expanded_list.append(item)
                continue

            for opt in rules[item[0].name].options:
                d = opt.conjuncts + item[1:]
                expanded_list.append(d)

        sentence_list = expanded_list
    return sentence_list

    # ----------------------------------------------------------------------------------------------------


def print_tree(root_node):
    """
    Prints cleanly the output of the grammar tree

    :param root_node: Root of the tree
    """

    work_list = [root_node]
    node_numbers = {}
    next_free_number = 1

    while work_list:
        node = work_list.pop()
        number = node_numbers.get(node)
        if not number:
            node_numbers[node] = next_free_number
            number = next_free_number
            next_free_number += 1
        print('{}:'.format(number))
        for edge in node.edges:
            number = node_numbers.get(edge.node)
            if not number:
                node_numbers[edge.node] = next_free_number
                edge_number = next_free_number
                next_free_number += 1
            print('   {} -> {}'.format(edge.word, number))
            work_list.append(edge.node)

    # ----------------------------------------------------------------------------------------------------


if __name__ == "__main__":
    import sys
    import pprint

    grammar_file = sys.argv[1]
    target = sys.argv[2]

    kaldi_gr = KaldiGrammar(grammar_file, target)
    tree = kaldi_gr.get_rule_element(target)
    pprint.pprint(tree)

    # first_words = kaldi_gr.get_first_words()
    # pprint.pprint(first_words)

    # # Get random sentence from the grammar
    # s = p.get_random_sentence('T')
    # print(s)
