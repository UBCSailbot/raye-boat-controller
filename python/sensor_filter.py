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
            Returns True if the reading is valid, and False otherwise.

        """

        # Check the type first so there is not a type error upon bound checking
        if not isinstance(reading, expected_type):
            return False

        bound_check = (lowerbound <= reading <= upperbound)
        return bound_check
