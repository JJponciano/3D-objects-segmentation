#include "interface.h"

float get_precision_input()
{
    float precision;

    std::cout << "> precision: ";
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

float get_z_min_input()
{
    float z_min;

    std::cout << "> z_min: ";
    std::cin >> z_min;
    clear_screen();

    return z_min;
}

float get_z_max_input()
{
    float z_max;

    std::cout << "> z_max: ";
    std::cin >> z_max;
    clear_screen();

    return z_max;
}

float get_float_input()
{
    float float_num;

    std::cout << "> float: ";
    std::cin >> float_num;
    clear_screen();

    return float_num;
}

float get_dec_precision_input()
{
    float precision;

    std::cout << "> dec_precision: ";
    std::cin >> precision;
    clear_screen();

    return precision;
}

float get_max_fragment_depth_input()
{
    float max_fragment_depth;

    std::cout << "> max_fragment_depth: ";
    std::cin >> max_fragment_depth;
    clear_screen();

    return max_fragment_depth;
}

int get_max_neighbs_input()
{
    float max_neighbs;

    std::cout << std::endl << "> max_neighbs: ";
    std::cin >> max_neighbs;
    clear_screen();

    return max_neighbs;
}

float get_epsilon_input()
{
    float epsilon;

    std::cout << "> epsilon: ";
    std::cin >> epsilon;
    clear_screen();

    return epsilon;
}

int get_file_type_input()
{
    int is_rgb;

    std::cout << "> is rgb (0 if false): ";
    std::cin >> is_rgb;
    clear_screen();

    return is_rgb;
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

    std::cout << "> export path (no extension): ";
    std::cin >> export_path_input;
    clear_screen();

    return export_path_input;
}

std::string get_magic_number_input()
{
    std::string magic_number;

    std::cout << "> magic_number (\"P2\" or \"P5\"): ";
    std::cin >> magic_number;
    clear_screen();

    return magic_number;
}

void clear_screen() { std::cout << std::string(50, '\n'); }

void invalid_input() { std::cout << "> Invalid input. Hit ENTER to continue..."; std::cin.get(); clear_screen(); }

void success() { std::cout << "> Test succeeded. Hit ENTER to continue..."; std::cin.ignore(); std::cin.get(); clear_screen(); }

void failure(char const* err)
{
    std::cout << "> Test failed. Error: " << err << " Hit ENTER to continue...";
    std::cin.ignore(); std::cin.get();
    clear_screen();
}

void test_menu()
{
    // user choice management
    int selection;
    bool exit;
    bool error;

    // export path
    std::string export_path;

    std::cout << "--- Normal Segmentation test library ---" << std::endl;

    do
    {
        error = false;

        std::cout << "1 - normal_estimation();" << std::endl;
        std::cout << "2 - e_normal_estimation();" << std::endl;
        std::cout << "3 - crop_cloud();" << std::endl;
        std::cout << "4 - color_to_grayscale();" << std::endl;
        std::cout << "5 - set_precision();" << std::endl;
        std::cout << "6 - cloud_homogenization();" << std::endl;
        std::cout << "7 - grayscale_to_image();" << std::endl;
        std::cout << "8 - image_to_cloud();" << std::endl;
        std::cout << "9 - railway_detection();" << std::endl;
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
                    export_path = NORMAL_ESTIMATION_RES_OUTPUT_PATH;

                    test_normal_estimation(get_import_path_input(), export_path, get_file_type_input(),
                                           get_radius_input(), get_max_neighbs_input());
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
                    export_path = E_NORMAL_ESTIMATION_RES_OUTPUT_PATH;

                    test_e_normal_estimation(get_import_path_input(), export_path, get_file_type_input(), get_radius_input(),
                                             get_max_neighbs_input(), get_xyz_input(), get_max_fragment_depth_input(),
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
                    export_path = CLOUD_CROP_RES_OUTPUT_PATH;

                    test_crop_cloud(get_import_path_input(), export_path, get_file_type_input(),
                                    get_xyz_input(), get_precision_input());
                    success();
                }

                catch (char const* err)
                {
                    failure(err);
                }
                break;

            case 4:
                try
                {
                    export_path = CONVERT_TO_GREYSCALE_RES_OUTPUT_PATH;

                    test_color_to_greyscale(get_import_path_input(), export_path, get_file_type_input());
                    success();
                }

                catch (char const* err)
                {
                    failure(err);
                }
                break;

            case 5:
                try
                {
                    std::cout << test_precision(get_float_input(), get_precision_input())
                              << std::endl;
                    success();
                }

                catch (std::logic_error err)
                {
                    failure(err.what());
                }
                break;

            case 6:
                try
                {
                    export_path = CLOUD_HOMOG_RES_OUTPUT_PATH;

                    test_cloud_homogenization(get_import_path_input(), export_path,
                                              get_file_type_input(), get_epsilon_input());
                    success();
                }

                catch(std::exception err)
                {
                    failure(err.what());
                }
                break;

            case 7:
                try
                {
                    export_path = IMAGE_TO_FILE_RES_OUTPUT_PATH;

                    test_greyscale_to_image(get_import_path_input(), export_path,
                                            get_file_type_input(), get_epsilon_input());
                    success();
                }

                catch(char const* err)
                {
                    failure(err);
                }
                break;

            case 8:
                try
                {
                    export_path = IMAGE_TO_CLOUD_RES_OUTPUT_PATH;

                    test_image_to_cloud(get_import_path_input(), export_path, get_file_type_input(),
                                        get_epsilon_input());
                    success();
                }

                catch(char const* err)
                {
                    failure(err);
                }
                break;

            case 9:
                try
                {
                    export_path = "/home/vlad-adrian/share/sem_detection/semanticDir/test/railway_detection_test_results/";
                    test_rail_detection(get_import_path_input(), export_path, get_file_type_input(),
                                        get_epsilon_input());
                    success();
                }

                catch(char const* err)
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
