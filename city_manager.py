from transformations import Transformation


class CityManager:

    def __init__(self):
        self.tour_mapping = {'ND': ['amsterdam', 'berlin', 'dresden', 'hamburg', 'koeln', 'leipzig', 'wuerzburg'],
                            'SF': ['barcelona', 'bologna', 'lyon', 'marseille', 'montpellier', 'toulouse'],
                            'D': ['basel', 'stuttgart', 'zuerich'],
                            'EE': ['bratislava', 'brno', 'budapest', 'ljubljana', 'nuernberg', 'prague', 'zagreb'],
                            'IT': ['firenze', 'milano', 'pisa', 'roma', 'torino'],
                            'DP': ['potsdam', 'szczecin']
                            }

        self.reverse_mapping = {}
        for tour, cities in self.tour_mapping.items():
            for c in cities:
                self.reverse_mapping[c] = tour

        self.transformations = {}


    def get_transformation_for_city(self, city):
        tour = self.reverse_mapping[city]
        if tour not in self.transformations:
            self.transformations[tour] = Transformation(tour)

        return self.transformations[tour]
