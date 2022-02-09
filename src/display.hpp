#include "main.h"

namespace display {

static const char * homeBtnMatrixMap[] = {"Auton Selection", "Pre-flight Setup", "\n",
					"Tests", "Graphs", "Calibration", "\n", ""};
lv_obj_t* homeBtnMatrix = lv_btnm_create(lv_scr_act(), NULL);

lv_obj_t* autonSelectorScr = lv_obj_create(NULL, NULL);
lv_obj_t* graphsScr = lv_obj_create(NULL, NULL);
lv_obj_t* calibrateScr = lv_obj_create(NULL, NULL);

auto defaultScr = lv_scr_act();
/*
lv_obj_t* autonSelectorBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* autonSelectorBtnLbl = lv_label_create(autonSelectorBtn, NULL);
lv_obj_t* autonSelectorScr = lv_obj_create(NULL, NULL);

lv_obj_t* preRunBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* preRunBtnLbl = lv_label_create(preRunBtn, NULL);

lv_obj_t* graphsBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* graphsBtnLbl = lv_label_create(graphsBtn, NULL);
lv_obj_t* graphsScr = lv_obj_create(NULL, NULL);

lv_obj_t* calibrateBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* calibrateBtnLbl = lv_label_create(calibrateBtn, NULL);
lv_obj_t* calibrateScr = lv_obj_create(NULL, NULL);
*/
lv_obj_t* homeBtn = lv_btn_create(lv_scr_act(), NULL);
lv_obj_t* homeBtnLbl = lv_label_create(homeBtn, NULL);

void autonSelectorBtnAction() {
	lv_scr_load(autonSelectorScr);
	lv_obj_set_parent(homeBtn, lv_scr_act());
	lv_obj_set_hidden(homeBtn, false);
	
}
void preRunBtnAction() {
	liftControl->setTarget(-17.75);
	liftControl->flipDisable(false);
	
}

void graphsBtnAction() {
	lv_scr_load(graphsScr);
	lv_obj_set_parent(homeBtn, lv_scr_act());
	lv_obj_set_hidden(homeBtn, false);
	
}

void calibrateBtnAction() {
	lv_scr_load(calibrateScr);
	lv_obj_set_parent(homeBtn, lv_scr_act());
	lv_obj_set_hidden(homeBtn, false);
	
}

lv_res_t homeBtnAction(lv_obj_t* btn) {
	lv_scr_load(defaultScr);
	lv_obj_set_hidden(homeBtn, true);
	return LV_RES_OK;
}

lv_res_t homeBtnMatrixHandler(lv_obj_t* obj, const char* txt) {
	std::string istr = txt;
	std::map<std::string, int> map { {"Auton Selection", 0}, {"Pre-flight Setup", 1}, {"Tests", 2}, {"Graphs", 3}, {"Calibration", 4}};
	switch(map.at(txt)) {
		case 0:
			autonSelectorBtnAction();
			break;
		case 1:
			preRunBtnAction();
			break;
		case 2:
			
			break;
		case 3:
			graphsBtnAction();
			break;
		case 4:
			calibrateBtnAction();
			break;

	}
	return LV_RES_OK;
}

void startScreen() {
	lv_theme_t* th = lv_theme_night_init(266, &lv_font_dejavu_20);
	lv_theme_set_current(th);

	lv_btnm_set_map(homeBtnMatrix, homeBtnMatrixMap);
	lv_btnm_set_action(homeBtnMatrix, homeBtnMatrixHandler);
	lv_obj_set_size(homeBtnMatrix, LV_HOR_RES - 10, 115);
	lv_obj_align(homeBtnMatrix, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);
	
	
	lv_obj_set_size(homeBtn, 55, 55);
	lv_obj_set_hidden(homeBtn, true);
	lv_obj_set_pos(homeBtn, 420, 181);
	lv_label_set_text(homeBtnLbl, SYMBOL_HOME);
	lv_btn_set_action(homeBtn, LV_BTN_ACTION_CLICK, homeBtnAction);
/*
	lv_obj_set_size(autonSelectorBtn, 240, 55);
	lv_obj_set_pos(autonSelectorBtn, 5, 5);
	lv_label_set_text(autonSelectorBtnLbl, "Autonomous Selection");
	lv_btn_set_action(autonSelectorBtn, LV_BTN_ACTION_CLICK, autonSelectorBtnAction);
	
	lv_obj_set_size(preRunBtn, 225, 55);
	lv_obj_set_pos(preRunBtn, 250, 5);
	lv_label_set_text(preRunBtnLbl, "Pre-Flight Setup");
	lv_btn_set_action(preRunBtn, LV_BTN_ACTION_CLICK, preRunBtnAction);

	lv_obj_set_size(graphsBtn, 160, 55);
	lv_obj_set_pos(graphsBtn, 5, 125);
	lv_label_set_text(graphsBtnLbl, "Graphs & Diags");
	lv_btn_set_action(graphsBtn, LV_BTN_ACTION_CLICK, graphsBtnAction);

	lv_obj_set_size(calibrateBtn, 160, 55);
	lv_obj_set_pos(calibrateBtn, 5, 185);
	lv_label_set_text(calibrateBtnLbl, "Calibration");
	lv_btn_set_action(calibrateBtn, LV_BTN_ACTION_CLICK, calibrateBtnAction);
*/	
}

} // namespace display
