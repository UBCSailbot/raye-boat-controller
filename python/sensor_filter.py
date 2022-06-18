class SensorFilter:

    @staticmethod
    def filter(reading, lowerbound, upperbound, expected_type):
        """
        Checks the validity of a reading. The function checks two things:
            1) That the reading has the expected type
            2) Checks that the reading is within the valid bounds

        Arguments
        =========
        Any : reading
            The reading to be checked.

        Any : lowerbound
            The lowerbound of the reading

        Any : upperbound
            The upperbound of the reading

        Any : expected_type
            The type of the reading.

        Returns
        =======
        bool
            Returns True if the reading is valid, and False otherwise.;

        """
        type_check = type(reading) is expected_type
        bound_check = (lowerbound <= reading <= upperbound)
        return type_check and bound_check
    