#! /usr/bin/env python

# Make python 2/3 compatible
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)
from builtins import *

import os
import shutil
import graphviz
from grammar_parser.cfgparser import CFGParser


class Grammar:
    """
    Class Grammar uses as input a grammar file with extension '.fcfg' and has two functions:
    get_rule_element: extracts the defined grammar rules
    get_words: extracts the unique words and creates 'corpus.txt' which is used to build 'G.fst'
    """
    def __init__(self, model_path, grammar_file_string, target):

        self.model_path = model_path
        self.model_path_tmp = os.path.join(self.model_path, "tmp")

        # If model_path exists, create a tmp directory in it
        if not os.path.exists(self.model_path):
            raise Exception("Model path '{}' does not exist".format(self.model_path))
        else:
            if os.path.exists(self.model_path_tmp):
                shutil.rmtree(self.model_path_tmp)

            os.mkdir(self.model_path_tmp)

        # Check if the grammar is a file or string and parse it
        if os.path.exists(grammar_file_string):
            self.parser = CFGParser.fromfile(grammar_file_string)
            self.grammar_file = grammar_file_string
            self.grammar_string = None
        else:
            self.parser = CFGParser.fromstring(grammar_file_string)
            self.grammar_file = None
            self.grammar_string = grammar_file_string

        self.target = target

        # Execute the following in the constructor
        self.get_words_()
        self.tree = self.expand_tree_()


    def get_words_(self):
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

        words = [word.upper() for word in list(words)]
        words.sort()

        # Create corpus.txt file and save the words list
        corpus_path = os.path.join(self.model_path_tmp, "corpus.txt")
        with open(corpus_path, "w") as f:
            for word in words:
                f.write(word + "\n")


    def autocomplete(self):
        """
        # TODO: expand the full tree, not only the first words
        # replace raw_input with the speech recognition output
        # add an option to skip a word if it is not a match and to check the
        # next word
        """
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


    def print_nicely(self, sentence_list):
        """
        Prints cleanly the output of the tree traversal functions

        :param sentence_list: list of possible completions
        """
        for sentence in sentence_list:
            line = [item for item in sentence]
            print(" ".join(line))
        print('')


    def expand_tree_(self):
        """
        Expands the grammar tree based on the words in the grammar rules for the
        pre-set target

        :return: tree of sentence nodes
        """
        # Extract rules from the grammar file
        rules = self.parser.rules
        return expand_tree(rules, self.target)

    def parse(self, sentence):
        """
        Parses the input sentence to generate the semantics for the pre-set
        target

        :param sentence: The sentence to be parsed
        :return: semantics
        """
        semantics = self.parser.parse(self.target, sentence)
        return semantics

    def print_graphviz(self):
        """
        Wrapper around the print_graphviz function to print the current tree
        """
        print_graphviz(self.tree, self.model_path_tmp)


class SentenceNode:
    """
    A node in a sentence.
    :ivar edges: Edges to the next node.
    :ivar done: Reached the end of the sentence.
    """
    def __init__(self):
        self.edges = []
        self.done = False


class SentenceEdge:
    """
    An edge in a sentence.
    :ivar word: The word to be understood.
    :ivar node: Node for the remainder of the sentence.
    """
    def __init__(self, word, node):
        self.word = word
        self.node = node


def expand_tree(rules, target='T'):
    """
    Expands the grammar tree based on the words in the grammar rules.

    :param rules: Extracted rules from the grammar file.
    :param target: Target rule to expand, default is 'T'.
    :return: The root of the expanded tree.
    :rtype: SentenceNode
    """
    # Map of set of successor rules to nodes.
    available_nodes = {}

    # Pairs of node and rule suffixes that need further work.
    work_list = []

    # Construct the initial node and the first set of suffix rules to expand further.
    root_list = [opt.conjuncts[:] for opt in rules[target].options]
    root_node = assign_node(root_list, available_nodes, work_list, rules)
    while work_list:
        node, expanded_list = work_list.pop()

        # collects alternatives on common prefixes and stores successor sentences
        prefix_dict = {}
        for item in expanded_list:
            successors = prefix_dict.get(item[0].name)
            if successors:
                # Store the expanded successor sentence in existing entry.
                successors.append(item[1:])
            else:
                # Store the expanded successor sentence found a non-existing prefix.
                prefix_dict[item[0].name] = [item[1:]]

        # Iterate over the collected prefixes and make a new edge for the words.
        for word, successors in prefix_dict.items():
            # Find the node to jump to after recognizing 'word'.
            nextnode = assign_node(successors, available_nodes, work_list, rules)
            edge = SentenceEdge(word, nextnode)
            node.edges.append(edge)

    return root_node



def expand_sentences(sentence_list, rules):
    """
    Expands the grammar rules until elimination of all variables at the first position

    :param sentence_list: List of grammar rules
    :param rules: Rules of the grammar
    :return: Expanded list, an whether an end of an sentence was found.
    """
    end_found = False
    while sentence_list:
        # decide if we need to expand anything
        not_expanded = False
        for item in sentence_list:
            # Need to remove all empty alternatives.
            if not item:
                not_expanded = True
                end_found = True
                continue

            # Found an alternative, that needs further expansion.
            if item[0].is_variable:
                not_expanded = True

        # All first enries are words already, done!
        if not not_expanded:
            break

        # Expand variables at the first entry.
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

    return end_found, sentence_list


def stringify_suffixes(expanded_list):
    """
    Convert the current rule suffixes to string form.

    :param expanded_list: List of rule suffixes to convert.
    :return: Set of suffixes, after converting each to a string.
    """
    sentence_set = set()
    for sentence in expanded_list:
        sentence_text = " ".join(conjunct.name for conjunct in sentence)
        sentence_set.add(sentence_text)
    return sentence_set


def assign_node(sentence_list, available_nodes, work_list, rules):
    """
    For a given list of rule suffixes, find or add a node, and update the work list if necessary.

    :param sentence_list: List of rule suffixes to find or add a node for.
    :type  sentence_list: List of rule alternatives (a list of conjuncts, partly expanded to words,
            in particular, the first conjuct shou d not be a variable).

    :param available_nodes: Known set of rule sufixes and their associated nodes. May be updated.
    :type  available_nodes: Dict of str to SentenceNode

    :param work_list: List or rule suffixes that need further processing. May be updated.
    :type  work_list: List of pairs (node, rule suffixes).

    :param rules: Rules of the grammar.

    :return: Node associated with the provided sentence_list.
    """
    end_found, sentence_list = expand_sentences(sentence_list, rules)
    sentence_set = stringify_suffixes(sentence_list)
    sentence_set = frozenset(sentence_set)
    node = available_nodes.get(sentence_set)
    if node is None:
        node = SentenceNode()
        node.done = end_found
        available_nodes[sentence_set] = node

        non_empty_sentences = []
        for sentence in sentence_list:
            if sentence:
                non_empty_sentences.append(sentence)
            else:
                node.done = True

        work_list.append((node, non_empty_sentences))
    return node


def print_graphviz(root_node, outpath):
    """
    Prints Graphviz input of the tree.

    :param root_node: Root of the tree
    """

    work_list = [root_node]
    node_numbers = {}
    printed_numbers = set()
    next_free_number = 1

    graphviz_dotfile_string = "digraph G {\n"

    while work_list:
        node = work_list.pop()
        number = node_numbers.get(node)
        if not number:
            node_numbers[node] = next_free_number
            number = next_free_number
            next_free_number += 1
        else:
            if number in printed_numbers:
                continue

        # Print the node.
        if node.done:
            shape = "box"
        else:
            shape = "ellipse"
        node_text = "node{}".format(number)
        printed_numbers.add(number)
        graphviz_dotfile_string += "{} [shape={}];".format(node_text, shape) \
                + "\n"

        # Print its edges.
        for edge in node.edges:
            number = node_numbers.get(edge.node)
            if not number:
                node_numbers[edge.node] = next_free_number
                number = next_free_number
                next_free_number += 1
            dest_text = "node{}".format(number)
            work_list.append(edge.node)
            graphviz_dotfile_string += "{} -> {} [label={}];".format(node_text,
                    dest_text, edge.word) + "\n"

    graphviz_dotfile_string += "}"

    # Print and render the graphviz file at the output location
    dotfile_path_noext = os.path.join(outpath, "grammar_tree")
    print(graphviz_dotfile_string, file=dotfile_path_noext + ".dot")
    graphviz.render('dot', 'pdf', dotfile_path_noext)

