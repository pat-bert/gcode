from BaseCmd import BaseCmd
from MelfaCmd import MelfaCmd
from typing import *


class LangDictEntry:
    """
    Implementation of one individual entry in a language dictionary which defines its translation.
    """

    def __init__(self, name: Union[str, Iterable[str]], output_cmd_id: str, translate_callback=(lambda x, *_: x)):
        """
        Define a language dictionary entry.
        :param name: Unique name for identification
        :param translate_callback: Function to convert entry, defaults to feed through second argument
        """
        self.name = name
        self.output_id = output_cmd_id
        self.translate_callback = translate_callback

    def transpose(self, input_obj: BaseCmd) -> BaseCmd:
        """
        Uses the entry to translate a command.
        :param input_obj: Input command object.
        :return:
        """
        return self.translate_callback(input_obj, self.name, self.output_id)


class LanguageDictionary:
    """
    Implementation of a whole language dictionary consisting of individual entries.
    The dictionary is independent of language definitions so different dictionaries can be used for the same language
    conversions.
    """

    def __init__(self, translation_entries: Union[LangDictEntry, Iterable[LangDictEntry]]) -> None:
        """
        Create the dictionary from individual elements for translations.
        :param translation_entries: List of translation descriptions.
        """
        self.entries = {}
        for e in translation_entries:
            self.add_entry(e)

    def add_entry(self, entry: LangDictEntry) -> None:
        """
        Add another entry.
        :param entry: Entry object or list of it
        """
        try:
            for key in entry.name:
                self.entries[key] = entry
                self.entries[key].name = key
        except TypeError:
            self.entries[entry.name] = entry

    def remove_entry(self, entry: LangDictEntry) -> Union[None, LangDictEntry]:
        """
        Remove an existing entry.
        :param entry: Command object
        :return:
        """
        return self.entries.pop(entry.name, None)

    def resolve(self, cmd: BaseCmd) -> BaseCmd:
        """
        Fetch the translation of a command using the dictionary
        :param cmd:
        :return:
        """
        try:
            # Look up the passed cmd using its id and call the transpose method
            return self.entries[cmd.id].transpose(cmd)
        except KeyError:
            # Unknown command or illegal access for dictionary
            raise

    def __add__(self, other):
        return LanguageDictionary([self.entries, other.entries])

    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)


class TransposeG2Melfa:
    """
    Class to collect all transposing functions from G-code to Melfa
    """
    CODE_ID = 'code_id'

    @classmethod
    def feed_through(cls, *args: Any) -> Any:
        """
        Feeds through the first argument and ignores the rest.
        :param args:
        :return:
        """
        mapping = {
            cls.CODE_ID: args[2]
        }
        return cls.init_melfa(mapping)

    @classmethod
    def ignore(cls, *_: Any) -> None:
        """
        Ignores the input and returns a comment as command.
        :param _: Pass anything
        :return: None
        """
        return MelfaCmd.read_cmd_str(MelfaCmd.COMMENT)

    @classmethod
    def init_melfa(cls, mapping: Mapping) -> MelfaCmd:
        return MelfaCmd(
            mapping[cls.CODE_ID]
        )


def create_entries_g2melfa() -> List[LangDictEntry]:
    entries = [
        # Linear interpolation
        LangDictEntry(['G0', 'G1'], '?', TransposeG2Melfa.feed_through),
        # Circular interpolation
        LangDictEntry('G2', '?', TransposeG2Melfa.feed_through),
        LangDictEntry('G3', '?', TransposeG2Melfa.feed_through)
    ]

    return entries


def create_lang_dict_g2melfa() -> LanguageDictionary:
    entries = create_entries_g2melfa()
    return LanguageDictionary(entries)
