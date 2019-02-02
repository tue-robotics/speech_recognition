import logging

import sys
import os
from Queue import Queue
from compiler.ast import flatten
from dragonfly import Alternative, Sequence, Literal, Grammar, Rule, Optional, Repetition
if os.name == 'nt':
    sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/grammar_parser/src/")
from grammar_parser.cfgparser import CFGParser

FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)

RULES = {}


def _get_dragonfly_rule_element(target, parser, depth=0):
    global RULES
    if target not in parser.rules:
        raise Exception("Target {} not in parser rules".format(target))

    # If already present in RULES, return it
    if target in RULES:
        return RULES[target]

    # Get the rule
    rule = parser.rules[target]

    # Iterate over all options
    option_alternative_list = []
    for opt in rule.options:

        # Iterate over all conjunctions
        conjunctions_list = []
        for conj in opt.conjuncts:
            # If the conjunction is already present
            if conj.name in RULES:
                conjunctions_list.append(RULES[conj.name])
                continue

            # If variable: go one level deeper
            if conj.is_variable:
                result = _get_dragonfly_rule_element(conj.name, parser, depth + 1)
                if result:
                    conjunctions_list.append(result)
            else:
                # Add a new literal to the list
                RULES[conj.name] = Literal(conj.name)
                conjunctions_list.append(RULES[conj.name])
                logger.debug("Adding literal rule: %s",  conj.name)

        # ToDo: apply caching?
        if len(conjunctions_list) == 1:
            option_alternative_list.append(conjunctions_list[0])
        else:
            option_alternative_list.append(Sequence(conjunctions_list))

    if len(option_alternative_list) == 1:
        RULES[target] = option_alternative_list[0]
    else:
        RULES[target] = Alternative(option_alternative_list)

    logger.debug("Adding alternative rule: %s", target)
    return RULES[target]


def get_dragonfly_grammar(grammar, target, result_queue):
    global RULES
    RULES = {}

    parser = CFGParser.fromstring(grammar)
    dragonfly_rule_element = _get_dragonfly_rule_element(target, parser)

    dragonfly_grammar = Grammar("G")
    with result_queue.mutex:
        result_queue.queue.clear()

    class GrammarRule(Rule):
        def process_recognition(self, node):
            logger.info('Dragonfly node: %s', str(node))
            result = node.value()
            logger.info('Dragonfly result: %s', str(result))

            flattened_string = result
            if isinstance(result, list):
                flattened_string = " ".join(flatten(result))

            logger.info('Dragonfly flattened result: %s', str(flattened_string))

            if not result_queue.empty():
                logger.warn('There is already a message in the queue! %s', result_queue)
            result_queue.put_nowait(flattened_string)

            logger.debug("Dragonfly thread recognition Q [id=%s, qsize=%d]", id(result_queue), result_queue.qsize())

    rule = GrammarRule(element=dragonfly_rule_element, exported=True)
    dragonfly_grammar.add_rule(rule)

    return dragonfly_grammar


