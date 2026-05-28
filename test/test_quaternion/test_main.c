#include <unity.h>
#include <math.h>
#include "../../lib/quaternion/quaternion.h"

void setUp(void) { }

void tearDown(void) { }

// ==========================================
// 1. TESTS DES OPERATIONS DE BASE
// ==========================================

void test_quaternion_addition(void) {
    quaternion q1 = {1.0f, 2.0f, 3.0f, 4.0f};
    quaternion q2 = {0.5f, -1.0f, 2.0f, -3.0f};
    quaternion out;

    quaternion_addition(&q1, &q2, &out);

    TEST_ASSERT_EQUAL_FLOAT(1.5f, out.w);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.x);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, out.y);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.z);
}

void test_quaternion_subtraction(void) {
    quaternion q1 = {1.0f, 2.0f, 3.0f, 4.0f};
    quaternion q2 = {0.5f, -1.0f, 2.0f, -3.0f};
    quaternion out;

    quaternion_subtraction(&q1, &q2, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.w);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, out.x);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.y);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, out.z);
}

void test_quaternion_scalar_product(void) {
    quaternion q = {1.0f, -2.0f, 3.5f, 0.0f};
    quaternion out;

    quaternion_scalar_product(&q, 2.0f, &out);

    TEST_ASSERT_EQUAL_FLOAT(2.0f, out.w);
    TEST_ASSERT_EQUAL_FLOAT(-4.0f, out.x);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, out.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.z);
}

void test_quaternion_product_identity(void) {
    quaternion q_identity = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion q = {0.5f, 0.5f, 0.5f, 0.5f};
    quaternion out;

    // Multiplier par l'identité ne doit pas modifier le quaternion
    quaternion_product(&q, &q_identity, &out);

    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.w);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.x);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.y);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, out.z);
}

void test_quaternion_product_rotation(void) {
    // Rotation de 90° autour de Z et 90° autour de Y
    quaternion q_rot_z = {0.7071f, 0.0f, 0.0f, 0.7071f};
    quaternion q_rot_y = {0.7071f, 0.0f, 0.7071f, 0.0f};
    quaternion out;

    quaternion_product(&q_rot_z, &q_rot_y, &out);

    // Valeurs théoriques attendues pour la composition
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, out.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, out.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, out.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, out.z);
}

// ==========================================
// 2. TESTS DES NORMES ET NORMALISATIONS
// ==========================================

void test_quaternion_norm(void) {
    quaternion q = {2.0f, 2.0f, 2.0f, 2.0f}; // sqrt(4+4+4+4) = sqrt(16) = 4
    float norm = quaternion_norm(&q);

    TEST_ASSERT_EQUAL_FLOAT(4.0f, norm);
}

void test_quaternion_normalize_standard(void) {
    quaternion q = {4.0f, 0.0f, 0.0f, 0.0f};
    quaternion out;

    quaternion_normalize(&q, &out);

    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.z);
}

void test_quaternion_normalize_zero_division(void) {
    quaternion q_null = {0.0f, 0.0f, 0.0f, 0.0f};
    quaternion out;

    // Doit retourner le quaternion identité par sécurité
    quaternion_normalize(&q_null, &out);

    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.z);
}

// ==========================================
// 3. TESTS CONVERTISSEURS GÉOMÉTRIQUES
// ==========================================

void test_quaternion_recover_axis_angle(void) {
    // Quaternion représentant une rotation de 90° autour de l'axe X
    // q = [ cos(45°), sin(45°), 0, 0 ] = [ 0.7071, 0.7071, 0, 0 ]
    quaternion q = {0.7071068f, 0.7071068f, 0.0f, 0.0f};
    axis_angle aa;
    float pi = acosf(-1);

    quaternion_recover_axis_angle(&q, &aa);

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (float)(pi / 2.0f), aa.angle);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, aa.v1); // Axe X
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, aa.v2); // Axe Y
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, aa.v3); // Axe Z
}

// ==========================================
// 4. TEST ALGORITHMIQUE : DESCENTE DE GRADIENT
// ==========================================

void test_compute_gradient_descent_correction_perfect_align(void) {
    // Cas 1 : L'ESP32 est parfaitement à plat, immobile, et l'estimation q est déjà parfaite
    quaternion q_flat = {1.0f, 0.0f, 0.0f, 0.0f}; 
    quaternion q_accel = {0.0f, 0.0f, 0.0f, 1.0f}; // L'accéléromètre lit [0, 0, 1] (1g vertical)
    quaternion out_gradient;

    compute_gradient_descent_correction(&q_flat, &q_accel, &out_gradient);

    // L'erreur fx, fy, fz doit être nulle. Le gradient calculé doit donc être nul (ou rester à 0)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out_gradient.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out_gradient.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out_gradient.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out_gradient.z);
}

void test_compute_gradient_descent_correction_with_error(void) {
    // Cas 2 : Le quaternion actuel croit qu'on est à plat [1, 0, 0, 0]
    // Mais le capteur est incliné et voit la gravité sur l'axe X : [1.0g, 0.0g, 0.0g]
    quaternion q_estimated = {1.0f, 0.0f, 0.0f, 0.0f};
    quaternion q_accel = {0.0f, 1.0f, 0.0f, 0.0f}; 
    quaternion out_gradient;

    compute_gradient_descent_correction(&q_estimated, &q_accel, &out_gradient);
    
    // De plus, d'après les équations de Madgwick, pour corriger une inclinaison autour de Y, 
    // le gradient doit générer une poussée forte sur l'axe Y du quaternion (out_gradient.y)
    TEST_ASSERT_NOT_EQUAL_FLOAT(0.0f, out_gradient.y);
}

// ==========================================
// EXECUTION DU RUNNER
// ==========================================

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Groupe 1 : Opérations de base
    RUN_TEST(test_quaternion_addition);
    RUN_TEST(test_quaternion_subtraction);
    RUN_TEST(test_quaternion_scalar_product);
    RUN_TEST(test_quaternion_product_identity);
    RUN_TEST(test_quaternion_product_rotation);

    // Groupe 2 : Normes
    RUN_TEST(test_quaternion_norm);
    RUN_TEST(test_quaternion_normalize_standard);
    RUN_TEST(test_quaternion_normalize_zero_division);

    // Groupe 3 : Géométrie & Fusion
    RUN_TEST(test_quaternion_recover_axis_angle);
    RUN_TEST(test_compute_gradient_descent_correction_perfect_align);
    RUN_TEST(test_compute_gradient_descent_correction_with_error);

    return UNITY_END();
}
