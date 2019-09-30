class Translator:
    """
    Implements the transitions possible from one code language to another.
    """

    def __init__(self, origin_language, target_language, mapping):
        """
        Set up the translator as a conduit for a specific translation job.
        :param origin_language: Language implementation of existing code
        :param target_language: Language implementation of target code
        :param mapping: Language dictionary for translation from origin to target language
        """
        self.src_lang = origin_language
        self.trg_lang = target_language
        self.map = mapping

    def translate(self, input_program, evaluate_str=False, out_delimiter=''):
        """
        Executes a translation process with the current setting on a program.
        :param input_program: Program consisting of command in the origin language.
        :param evaluate_str: Optional flag to convert commands to string representation internally. If not set the
        objects for the output commands are returned.
        :param out_delimiter: Optional delimiter for merging the translated commands, default =''
        :return: Merged program as string
        """
        # Check logic of input
        self.src_lang.validate(input_program)

        # Translate individual commands
        output_program = []
        for command in input_program:
            output_program.append(self.map.resolve(command))

        # Merge the individual translated commands into a program
        if evaluate_str:
            return out_delimiter.join([str(cmd) for cmd in output_program])
        else:
            return output_program
