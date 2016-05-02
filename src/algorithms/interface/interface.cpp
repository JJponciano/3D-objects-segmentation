#include "interface.h"

float get_precision_input()
{
    float precision;

    std::cout << "> precision ";
    std::cin >> precision;
    clear_screen();

    return precision;
}

float get_radius_input()
{
    float radius;

    std::cout << "> radius: ";
    std::cin >> radius;
    clear_screen();

    return radius;
}

int get_max_neighbs_input()
{
    float max_neighbs;

    std::cout << std::endl << "> max_neighbs: ";
    std::cin >> max_neighbs;
    clear_screen();

    return max_neighbs;
}

std::vector<float> get_xyz_input()
{
    float x_scale;
    float y_scale;
    float z_scale;
    std::vector<float> xyzscale_input;

    std::cout << "> x: ";
    std::cin >> x_scale;
    std::cout << std::endl << "> y: ";
    std::cin >> y_scale;
    std::cout << std::endl << "> z: ";
    std::cin >> z_scale;
    clear_screen();

    xyzscale_input.push_back(x_scale);
    xyzscale_input.push_back(y_scale);
    xyzscale_input.push_back(z_scale);

    return xyzscale_input;
}

std::string get_import_path_input()
{
    std::string import_path_input;

    std::cout << "> import path: ";
    std::cin >> import_path_input;
    clear_screen();

    return import_path_input;
}

std::string get_export_path_input()
{
    std::string export_path_input;

    std::cout << "> export directory path: ";
    std::cin >> export_path_input;
    clear_screen();

    return export_path_input;
}

void clear_screen() { std::cout << std::string(50, '\n'); }

void invalid_input() { std::cout << "> Invalid input. Hit ENTER to continue..."; std::cin.ignore(); std::cin.get(); clear_screen(); }

void success() { std::cout << "> Test succeeded. Hit ENTER to continue..."; std::cin.ignore(); std::cin.get(); clear_screen(); }

void failure(char const* err)
{
    std::cout << "> Test failed. Error: " << err << " Hit ENTER to continue...";
    std::cin.ignore(); std::cin.get();
    clear_screen();
}

void test_menu()
{
    int selection;
    bool exit;
    bool error;

    std::cout << "--- Normal Segmentation test library ---" << std::endl;

    do
    {
        std::cout << "1 - normal_estimation();" << std::endl;
        std::cout << "2 - e_normal_estimation();" << std::endl;
        std::cout << "3 - crop_cloud();" << std::endl;
        std::cout << "0 - quit." << std::endl;
        std::cout << std::endl << "Your selection: ";
        std::cin >> selection;
        clear_screen();

        switch (selection)
        {
            case 0:
                exit = true;
                break;

            case 1:
                try
                {
                    test_normal_estimation(get_import_path_input(),
                                           get_export_path_input(),
                                           get_radius_input(),
                                           get_max_neighbs_input());
                    success();
                }

                catch (char const* err)
                {
                    failure(err);
                }

                break;

            case 2:
                try
                {
                    test_e_normal_estimation(get_import_path_input(),
                                             get_export_path_input(),
                                             get_radius_input(),
                                             get_max_neighbs_input(),
                                             get_xyz_input(),
                                             get_precision_input());
                    success();
                }

                catch (char const* err)
                {
                    failure(err);
                }

                break;

            case 3:
                try
                {
                    test_crop_cloud(get_import_path_input(),
                                    get_export_path_input(),
                                    get_xyz_input(),
                                    get_precision_input());
                    success();
                }

                catch (char const* err)
                {
                    failure(err);
                }

                break;

            default:
                error = true;
                invalid_input();
                break;
        }
    } while (!exit && !error);
}
