class LanguageDictionary:
    """
    Implementation of a whole language dictionary consisting of individual entries.
    The dictionary is independent of language definitions so different dictionaries can be used for the same language
    conversions.
    """

    def __init__(self, translation_entries):
        """
        Create the dictionary from individual elements for translations.
        :param translation_entries: List of translation descriptions.
        """
        self.entries = {e.name: e for e in translation_entries}

    def add_entry(self, entry):
        """
        Add another entry.
        :param entry: Command object
        """
        self.entries[entry.name] = entry

    def remove_entry(self, entry):
        """
        Remove an existing entry.
        :param entry: Command object
        :return:
        """
        return self.entries.pop(entry.name, None)

    def resolve(self, cmd):
        """
        Fetch the translation of a command using the dictionary
        :param cmd:
        :return:
        """
        try:
            # Look up the passed cmd using its id and call the transpose method
            self.entries[cmd.id].transpose(cmd)
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


class LangDictEntry:
    """
    Implementation of one individual entry in a language dictionary which defines its translation.
    """

    def __init__(self, name, transpose):
        """
        Define a language dictionary entry.
        :param name: Unique name for identification
        :param transpose: Function to convert entry
        """
        self.name = name
        self.transpose = transpose
